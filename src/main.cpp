#include <atomic>
#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>


#include "Core/Octree.hpp"
#include "Core/State.hpp"
#include "Core/Cloud.hpp"
#include "Core/Imu.hpp"

#include "Utils/Config.hpp"
#include "ROSutils.hpp"


class Manager {
  State  state_;
  States state_buffer_;
  
  Imu prev_imu_;
  double first_imu_stamp_;

  bool imu_calibrated_;

  std::mutex mtx_state_;
  std::mutex mtx_buffer_;

  std::condition_variable cv_prop_stamp_;

  charlie::Octree ioctree_;
  bool stop_ioctree_update_;

  ros::Publisher pub_state_, 
                 pub_frame_,
                 pub_raw_, 
                 pub_deskewed_, 
                 pub_downsampled_, 
                 pub_filtered_;

  tf2_ros::TransformBroadcaster br;
  ros::Subscriber initialpose_sub_;
  ros::Subscriber wheel_odom_sub_;

  // 缓存最新 wheel odom, 供 initialpose_callback 重置 state 用
  std::mutex mtx_wheel_odom_;
  nav_msgs::Odometry latest_wheel_odom_;
  bool wheel_odom_received_;

  // 重定位后第一帧 LiDAR 标志: 仅用于日志/Δwheel 校验。
  // 真正的 "空地图 update noop" 由 State::update() 内 `if (map.size()==0) return;` 兜底,
  // 这里只是为了让日志能明确指出 "这一帧发生在 reloc 之后", 方便事后排查。
  std::atomic<bool> relocalizing_after_clear_{false};
  ros::Time last_initialpose_time_;



public:
  Manager(ros::NodeHandle& nh) : first_imu_stamp_(-1.0), 
                                 state_buffer_(1000), 
                                 stop_ioctree_update_(false),
                                 wheel_odom_received_(false),
                                 ioctree_() {

    Config& cfg = Config::getInstance();

    imu_calibrated_ = not (cfg.sensors.calibration.gravity_align
                           or cfg.sensors.calibration.accel
                           or cfg.sensors.calibration.gyro)
                      or cfg.sensors.calibration.time <= 0.; 

    ioctree_.setBucketSize(cfg.ioctree.bucket_size);
    ioctree_.setDownsample(cfg.ioctree.downsample);
    ioctree_.setMinExtent(cfg.ioctree.min_extent);

    // Publishers
    pub_state_ = nh.advertise<nav_msgs::Odometry>(cfg.topics.output.state, 10);
    pub_frame_ = nh.advertise<sensor_msgs::PointCloud2>(cfg.topics.output.frame, 10);

    // Debug only
    pub_raw_         = nh.advertise<sensor_msgs::PointCloud2>("debug/raw",         10);
    pub_deskewed_    = nh.advertise<sensor_msgs::PointCloud2>("debug/deskewed",    10);
    pub_downsampled_ = nh.advertise<sensor_msgs::PointCloud2>("debug/downsampled", 10);
    pub_filtered_    = nh.advertise<sensor_msgs::PointCloud2>("debug/filtered",    10);

    // /initialpose: 重定位触发 -> 清地图+重置state+放大P, 让IESKF从新位置重收敛
    initialpose_sub_ = nh.subscribe("/initialpose", 1,
        &Manager::initialpose_callback, this);

    // 订阅 wheel odom, 仅缓存最新帧供 initialpose 重置用
    wheel_odom_sub_ = nh.subscribe("/odom_wheel", 10,
        &Manager::wheel_odom_callback, this, ros::TransportHints().tcpNoDelay());

    if (cfg.health_monitor)
      ROS_INFO("[LIMO] Health monitor ENABLED (grep [LIMO][health] / [LIMO][diverge])");
  };
  
  ~Manager() = default;


  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {

    Config& cfg = Config::getInstance();

    Imu imu = fromROS(msg);

    if (first_imu_stamp_ < 0.)
      first_imu_stamp_ = imu.stamp;
    
    // IMU 校准: 前一段时间内累积平均值, 作为零偏初始值; 可选重力对齐
    if (not imu_calibrated_) {
      static int N(0);
      static Eigen::Vector3d gyro_avg(0., 0., 0.);
      static Eigen::Vector3d accel_avg(0., 0., 0.);
      // 注意: 这里的时间是 IMU 自带的时间戳,如果 IMU 时间不连续或不正确,可能导致校准失败
      if ((imu.stamp - first_imu_stamp_) < cfg.sensors.calibration.time) {
        gyro_avg  += imu.ang_vel;
        accel_avg += imu.lin_accel; 
        N++;

      } else {
        // 计算平均值
        gyro_avg /= N;
        accel_avg /= N;

        if (cfg.sensors.calibration.gravity_align) {
          //重力对齐: 以平均加速度方向为重力方向,调整初始姿态使其与全局坐标系对齐
          Eigen::Vector3d g_m = (accel_avg - state_.b_a()).normalized(); 
                          g_m *= cfg.sensors.extrinsics.gravity;
          
          Eigen::Vector3d g_b = state_.quat().conjugate() * state_.g();
          Eigen::Quaterniond dq = Eigen::Quaterniond::FromTwoVectors(g_b, g_m);

          state_.quat((state_.quat() * dq).normalized());
        }
        
        // 陀螺仪和加速度计零偏校准
        if (cfg.sensors.calibration.gyro)
          state_.b_w(gyro_avg);

        if (cfg.sensors.calibration.accel)
          state_.b_a(accel_avg - state_.R().transpose()*state_.g());

        prev_imu_ = imu;  // 初始化 prev_imu_，避免下一帧 dt 计算异常
        imu_calibrated_ = true;
      }

    } else {
      double dt = imu.stamp - prev_imu_.stamp;

      if (dt < 0)
        ROS_ERROR("IMU timestamps not correct");

      //时间戳检查: 如果 IMU 时间不连续或跳变过大,可能导致状态预测异常,这里简单地限制 dt 的范围,避免跳到未来或过度预测
      dt = (dt < 0 or dt >= imu.stamp) ? 1./cfg.sensors.imu.hz : dt;

      // Correct acceleration
      //校正加速度计的系统误差:这里只是用简单的比例因子做的矫正
      imu.lin_accel = cfg.sensors.intrinsics.sm * imu.lin_accel;
      prev_imu_ = imu;

      //用校准后的 IMU 数据预测状态,并将预测结果存入缓冲区,供 LiDAR 回调使用 (位置,速度,姿态等)
      mtx_state_.lock();
        state_.predict(imu, dt);
      mtx_state_.unlock();

      mtx_buffer_.lock();
        state_buffer_.push_front(state_);
      mtx_buffer_.unlock();

      cv_prop_stamp_.notify_one();

      // 以 IMU 频率(~200Hz)发布里程计和 TF, 是系统的高频位姿输出
      pub_state_.publish(toROS(state_, imu.stamp));
      publishTFs(state_, br, imu.stamp);
    }
  }


  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    Config& cfg = Config::getInstance();

    //检查IMU数据是否完成校准,如果没有校准完成,则不处理 LiDAR 数据,避免使用不可靠的状态预测结果
    if (not imu_calibrated_)
      return;
    
    //检查是否收到处理后的IMU数据
    if (state_buffer_.empty()) {
      ROS_ERROR("[LIMONCELLO] No IMUs received");
      return;
    }
    
    // 原始点云数据结构: PointCloudT 通常是 pcl::PointCloud<PointT>,
    // 这里用智能指针保存从 ROS PointCloud2 转换后的整帧 LiDAR 点云
    PointCloudT::Ptr raw(boost::make_shared<PointCloudT>());
    fromROS(*msg, *raw);

    // 点云有效性检查: 如果转换后没有任何点,说明当前帧无效,直接丢弃
    if (raw->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Raw PointCloud is empty!");
      return;
    }

    //获取点云中每个点的时间戳,并计算整帧点云的起止时间,以便后续与状态缓冲区中的状态进行时间对齐和插值
    PointTime point_time = point_time_func();
    double sweep_time = msg->header.stamp.toSec();
    
    //估计点云和IMU状态之间的时间偏移
    //注意: 对于 Livox 自带 IMU + Livox 点云这类同源时间戳数据,通常已经完成硬件级同步,
    //一般不建议开启 cfg.sensors.time_offset,否则可能重复修正并引入额外误差。
    //只有在实际观测到 IMU/点云存在稳定时延时,才建议开启该选项做粗略补偿。
    double offset = 0.0;
    if (cfg.sensors.time_offset) { // automatic sync (not precise!)
      offset = state_.stamp - point_time(raw->points.back(), sweep_time) - 1.e-4; 
      if (offset > 0.0) offset = 0.0; // don't jump into future
    }

    // 计算当前整帧点云的起止时间:
    // 后面会用这个时间区间去状态缓冲区里做插值/去畸变。
    double start_stamp = point_time(raw->points.front(), sweep_time) + offset;
    double end_stamp   = point_time(raw->points.back(), sweep_time) + offset;

    // 如果状态缓冲区里最新可用状态的时间 still 早于本帧点云结束时间,
    // 说明 IMU/传播线程还没有把状态推到这帧扫描结束位置,
    // 此时先阻塞等待,直到 buffer 覆盖到 end_stamp 为止。
    if (state_buffer_.front().stamp < end_stamp) {
      double initial_gap_ms = (end_stamp - state_buffer_.front().stamp) * 1000.0;
      ROS_DEBUG_THROTTLE(2.0, "[LIMONCELLO] Propagate waiting: buffer=%.6f end_scan=%.6f gap=%.3fms",
                         state_buffer_.front().stamp, end_stamp, initial_gap_ms);

      auto t_wait_start = ros::WallTime::now();
      std::unique_lock<decltype(mtx_buffer_)> lock(mtx_buffer_);
      // 等待状态数据准备好。
      cv_prop_stamp_.wait(lock, [this, &end_stamp] { 
          return state_buffer_.front().stamp >= end_stamp;
      });
      double wait_ms = (ros::WallTime::now() - t_wait_start).toSec() * 1000.0;
      if (wait_ms > 200.0) {
        ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] propagate wait too long: %.1fms "
                          "(initial_gap=%.1fms) - IMU feed can't keep up with LiDAR",
                          wait_ms, initial_gap_ms);
      }
    } 


  mtx_buffer_.lock();
    States interpolated = filter_states(state_buffer_, start_stamp, end_stamp);
  mtx_buffer_.unlock();

    //插值检查
    if (start_stamp < interpolated.front().stamp or interpolated.size() == 0) {
      // every point needs to have a state associated not in the past
      ROS_WARN("Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

  mtx_state_.lock();//加锁:防止IMU回调同时修改state_ 

    //去畸变: 把每个点从它被采集时的局部坐标系，统一变换到当前帧结束时的坐标系。
    PointCloudT::Ptr deskewed    = deskew(raw, state_, interpolated, offset, sweep_time);
    //下采样和滤波: 减少点数，降低后续计算量，同时保持空间分布均匀
    PointCloudT::Ptr downsampled = voxel_grid(deskewed);
    //滤波: 根据配置的条件过滤掉一些点,比如距离过远的点,或者在某些区域内的点,以提高地图质量和匹配效率
    PointCloudT::Ptr filtered    = filter(downsampled, state_.isometry() * state_.L2I_isometry());// LiDAR 坐标系 → 全局坐标系 的变换

    // 滤波结果检查: 如果滤波后没有任何点,说明当前帧无效,直接丢弃
    if (filtered->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Filtered & downsampled cloud is empty!");
      mtx_state_.unlock();
      return;
    }

    // IESKF 观测更新: 用点云与地图匹配, 迭代修正位姿/bias/重力
    // 注: State::update() 内部对 map.size()==0 已 early-return, 所以重定位后第一帧
    //     update 是 noop, P 不会被错误收缩。这里只打印一行确认日志便于排查。
    bool first_frame_after_reloc = relocalizing_after_clear_.exchange(false);
    if (first_frame_after_reloc) {
      // Δwheel = 当前 state(base_link 系) 与 wheel_odom 的水平偏差
      // reset 时刚把 state.p 设成 wheel_odom 位置, 这里应该非常小; 若 > 0.5m 说明
      // imu2baselink 外参或 reset 逻辑有问题
      Eigen::Vector3d p_now_I = state_.p();
      Eigen::Isometry3d T_M_I_now = Eigen::Isometry3d::Identity();
      T_M_I_now.translation() = p_now_I;
      T_M_I_now.linear() = state_.quat().toRotationMatrix();
      Eigen::Isometry3d T_M_B_now = T_M_I_now * cfg.sensors.extrinsics.imu2baselink.inverse();
      double bx = T_M_B_now.translation().x();
      double by = T_M_B_now.translation().y();

      double wx = 0, wy = 0; bool has_w = false;
      {
        std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
        if (wheel_odom_received_) {
          has_w = true;
          wx = latest_wheel_odom_.pose.pose.position.x;
          wy = latest_wheel_odom_.pose.pose.position.y;
        }
      }
      double dwheel = has_w ? std::hypot(bx - wx, by - wy) : -1.0;
      ROS_WARN("[LIMO][first-frame] AFTER reloc: state.base=[%.3f %.3f] wheel=[%.3f %.3f] "
               "Δwheel=%.3fm | map.size=0 → update is noop, building map at this pose | "
               "filtered_pts=%zu",
               bx, by, wx, wy, dwheel, filtered->points.size());
      if (has_w and dwheel > 0.5) {
        ROS_ERROR("[LIMO][first-frame] Δwheel=%.3fm > 0.5m: reset position likely WRONG "
                  "(check imu2baselink extrinsics or wheel_odom freshness)", dwheel);
      }
    }

    state_.update(filtered, ioctree_);

    Eigen::Isometry3f T = (state_.isometry() * state_.L2I_isometry()).cast<float>();

    PointCloudT::Ptr to_save(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*filtered, *to_save, T);

    // Update map (must be under mtx_state_ to avoid race with initialpose_callback clear())
    if (not stop_ioctree_update_)
      ioctree_.update(to_save->points);

  mtx_state_.unlock();

    PointCloudT::Ptr global(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*deskewed, *global, T);

    // Publish
    pub_state_.publish(toROS(state_, sweep_time));
    pub_frame_.publish(toROS(global, sweep_time));

    if (cfg.debug) {
      pub_raw_.publish(toROS(raw, sweep_time));
      pub_deskewed_.publish(toROS(deskewed, sweep_time));
      pub_downsampled_.publish(toROS(downsampled, sweep_time));
      pub_filtered_.publish(toROS(to_save, sweep_time));
    }

    if (cfg.verbose)
      PROFC_PRINT()

    // ---------- Health monitor & divergence warnings ----------
    // grep tags: [LIMO][health] (5s periodic), [LIMO][diverge] (1s threshold alerts)
    // Controlled by param ~health_monitor (default: false)
    if (cfg.health_monitor) {
      const Eigen::Vector3d p  = state_.p();
      const Eigen::Vector3d v  = state_.v();
      const Eigen::Vector3d ba = state_.b_a();
      const Eigen::Vector3d bw = state_.b_w();
      const double p_norm  = p.norm();
      const double v_norm  = v.norm();
      const double ba_norm = ba.norm();
      const double bw_norm = bw.norm();
      const size_t map_sz  = ioctree_.size();

      // 与 wheel_odom 的短期分裂指标 (在 base_link 系下比, 跟 first-frame 日志一致)
      double dwheel = -1.0;
      {
        std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
        if (wheel_odom_received_) {
          Eigen::Isometry3d T_M_I_now = Eigen::Isometry3d::Identity();
          T_M_I_now.translation() = p;
          T_M_I_now.linear() = state_.quat().toRotationMatrix();
          Eigen::Isometry3d T_M_B_now = T_M_I_now * cfg.sensors.extrinsics.imu2baselink.inverse();
          dwheel = std::hypot(
              T_M_B_now.translation().x() - latest_wheel_odom_.pose.pose.position.x,
              T_M_B_now.translation().y() - latest_wheel_odom_.pose.pose.position.y);
        }
      }

      ROS_INFO_THROTTLE(5.0,
          "[LIMO][health] p=[%.2f %.2f %.2f] |p|=%.2f |v|=%.3f |b_a|=%.3f |b_w|=%.4f "
          "Δwheel=%.3fm map=%zu stop_update=%d",
          p.x(), p.y(), p.z(), p_norm, v_norm, ba_norm, bw_norm,
          dwheel, map_sz, (int)stop_ioctree_update_);

      // 发散阈值 (与 /memories/repo/limoncello_failure_modes.md "情况 B" 经验相关)
      //   p_norm > 100m : 一般室内场景几乎不可能
      //   v_norm > 5m/s: AGV 物理上限
      //   b_a > 5 m/s²  : 远超合理 IMU 零偏量级 (典型 < 0.5)
      //   b_w > 0.5 rad/s: 同上 (典型 < 0.05)
      //   Δwheel > 5m   : LIMOncello 与 wheel_odom 长期分裂
      if (p_norm  > 100.0) ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] state.p large: |p|=%.2f → likely diverging", p_norm);
      if (v_norm  >   5.0) ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] state.v abnormal: |v|=%.3f", v_norm);
      if (ba_norm >   5.0) ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] accel bias abnormal: |b_a|=%.3f", ba_norm);
      if (bw_norm >   0.5) ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] gyro  bias abnormal: |b_w|=%.4f", bw_norm);
      if (dwheel  >   5.0) ROS_WARN_THROTTLE(1.0, "[LIMO][diverge] LIMOncello vs wheel diverged: Δwheel=%.2fm", dwheel);
    }
  }


  void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
    latest_wheel_odom_ = *msg;
    wheel_odom_received_ = true;
  }


  void stop_update_callback(const std_msgs::Bool::ConstPtr& msg) {
    if (not stop_ioctree_update_ and msg->data) {
      stop_ioctree_update_ = msg->data;
      ROS_INFO("Stopping ioctree updates from now onwards");
    }
  }


  // /initialpose 重定位回调
  //
  // 完整动作 (执行顺序, 全部在同一 mtx_state_ + mtx_buffer_ 临界区内):
  //   1. 用 wheel odom 的当前位姿重置 state (p, quat), 速度清零
  //      → 保证从一个合理的物理位置重建地图
  //   2. 重置 b_a/b_w/g 回 yaml 默认值
  //      → 发散态下 bias 已被推到非物理值, 不重置则下一帧 IMU predict 立即把 state 推飞
  //      → 这是 "重定位后跑飞" 的最致命根因
  //   3. 清空 iOctree (state 飞了的时候点云被变换到 ~1e7 量级位置, octree 已污染)
  //   4. 重置 P (放大 100, 让 IESKF 前几帧高度信任 LiDAR)
  //   5. 清空 state_buffer_ + push 一个 dummy state(stamp=now), 避免 lidar_callback
  //      读 front() 时遇空 circular_buffer 触发 UB
  //   6. 置 relocalizing_after_clear_ 标志, 用于第一帧诊断
  //
  // 前提 (运维侧保证, 不在代码中做门控): 重定位时机器人静止
  void initialpose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    auto t_start = ros::WallTime::now();
    Config& cfg = Config::getInstance();

    // ---------- Step 0: 取 wheel_odom 快照 + 入口日志 ----------
    bool has_wheel = false;
    double wh_x = 0, wh_y = 0, wh_yaw = 0;
    double wh_age = -1.0;
    double wh_vx = 0, wh_wz = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
      if (wheel_odom_received_) {
        has_wheel = true;
        wh_x = latest_wheel_odom_.pose.pose.position.x;
        wh_y = latest_wheel_odom_.pose.pose.position.y;
        const auto& oq = latest_wheel_odom_.pose.pose.orientation;
        wh_yaw = std::atan2(2.0*(oq.w*oq.z + oq.x*oq.y),
                            1.0 - 2.0*(oq.y*oq.y + oq.z*oq.z));
        wh_vx = latest_wheel_odom_.twist.twist.linear.x;
        wh_wz = latest_wheel_odom_.twist.twist.angular.z;
        wh_age = (ros::Time::now() - latest_wheel_odom_.header.stamp).toSec();
      }
    }

    // 入口日志
    ROS_WARN("[LIMO][relocalize] ENTER /initialpose target=(%.3f, %.3f) frame=%s | "
             "wheel_odom: %s age=%.3fs (x=%.3f y=%.3f yaw=%.2fdeg vx=%.3f wz=%.3f)",
             msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->header.frame_id.c_str(),
             has_wheel ? "YES" : "NO",
             wh_age, wh_x, wh_y, wh_yaw * 180.0 / M_PI, wh_vx, wh_wz);

    // 运维提醒: 不做硬门控, 但若检测到运动则提示, 方便排查
    if (has_wheel and (std::abs(wh_vx) > 0.05 or std::abs(wh_wz) > 0.05)) {
      ROS_WARN("[LIMO][relocalize] robot NOT stationary at reloc (vx=%.3f wz=%.3f), "
               "recovery may degrade. Recommend stopping before /initialpose.",
               wh_vx, wh_wz);
    }

    // ---------- Step 1-5: reset (持 mtx_state_ + mtx_buffer_, 顺序固定避免死锁) ----------
    Eigen::Vector3d old_p, old_v, old_ba, old_bw, old_g;
    Eigen::Quaterniond old_q;
    Eigen::Vector3d new_p, new_v, new_ba, new_bw, new_g;
    Eigen::Quaterniond new_q;
    double imu_stamp_used = 0.0;

    std::lock(mtx_state_, mtx_buffer_);
    std::lock_guard<std::mutex> lk_s(mtx_state_, std::adopt_lock);
    std::lock_guard<std::mutex> lk_b(mtx_buffer_, std::adopt_lock);

      // 1) 记录 BEFORE
      old_p  = state_.p();
      old_v  = state_.v();
      old_q  = state_.quat();
      old_ba = state_.b_a();
      old_bw = state_.b_w();
      old_g  = state_.g();

      // 2) 重置位姿 + 速度
      if (has_wheel) {
        // 用 wheel odom 的 2D 位姿重置 state (Z=0, roll/pitch=0; bias 重置后 g 对齐会处理倾角)
        Eigen::Isometry3d T_M_B = Eigen::Isometry3d::Identity();
        T_M_B.translation() = Eigen::Vector3d(wh_x, wh_y, 0.0);
        T_M_B.linear() = Eigen::AngleAxisd(wh_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        // state 存的是 IMU 帧位姿, 需要 T_M_I = T_M_B * T_B_I
        Eigen::Isometry3d T_M_I = T_M_B * cfg.sensors.extrinsics.imu2baselink;

        state_.p(T_M_I.translation());
        state_.quat(Eigen::Quaterniond(T_M_I.linear()).normalized());
        state_.v(Eigen::Vector3d::Zero());
      } else {
        // wheel odom 还没收到, 只能把速度清零, 位置不动
        state_.v(Eigen::Vector3d::Zero());
        ROS_WARN("[LIMO][relocalize] no wheel_odom yet, position NOT reset (v=0 only)");
      }

      // 3) 【关键】重置 bias 和 g 回 yaml 默认值
      //    出错场景下 bias 已发散; 不重置则下一帧 predict 立即再飞
      state_.b_w(cfg.sensors.intrinsics.gyro_bias);
      state_.b_a(cfg.sensors.intrinsics.accel_bias);
      state_.g(Eigen::Vector3d::UnitZ() * cfg.sensors.extrinsics.gravity);

      // 4) 重置协方差 (×100 而非 ×1000, 配合第一帧 skip update 已足够让 IESKF 高信任 LiDAR)
      constexpr double P_RELOC_GAIN = 100.0;
      state_.P.setIdentity();
      state_.P *= cfg.ikfom.covariance.initial_cov * P_RELOC_GAIN;

      // 5) 清地图 + 清 buffer
      ioctree_.clear();
      state_buffer_.clear();

      //    push 一个 dummy state 防 lidar_callback front() 读空 UB
      //    stamp=now, 这样 lidar_callback 的 `front().stamp >= end_scan` 检查会立即通过
      state_.stamp = ros::Time::now().toSec();
      imu_stamp_used = state_.stamp;
      state_buffer_.push_front(state_);

      // 6) 标记: 下一帧 LiDAR 触发首帧诊断日志
      relocalizing_after_clear_.store(true);
      last_initialpose_time_ = ros::Time::now();

      // 同临界区取 AFTER (旧实现是 unlock 后再 lock 二次读, 值可能已被 lidar_callback 改写)
      new_p  = state_.p();
      new_v  = state_.v();
      new_q  = state_.quat();
      new_ba = state_.b_a();
      new_bw = state_.b_w();
      new_g  = state_.g();

    // 释放锁 (lk_s/lk_b 析构)

    // ---------- 完整诊断日志 ----------
    double dt_ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;
    ROS_WARN("[LIMO][relocalize] DONE in %.1fms\n"
             "  BEFORE p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] |v|=%.3f "
             "|b_a|=%.3f |b_w|=%.4f |g|=%.3f q=[w=%.4f xyz=(%.4f %.4f %.4f)]\n"
             "  AFTER  p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] |v|=%.3f "
             "|b_a|=%.3f |b_w|=%.4f |g|=%.3f q=[w=%.4f xyz=(%.4f %.4f %.4f)]\n"
             "  wheel=%s (x=%.3f y=%.3f yaw=%.2fdeg) | P*=%.1f | buffer pushed dummy@%.6f | "
             "ioctree CLEARED | relocalizing_after_clear_=true",
             dt_ms,
             old_p.x(), old_p.y(), old_p.z(),
             old_v.x(), old_v.y(), old_v.z(), old_v.norm(),
             old_ba.norm(), old_bw.norm(), old_g.norm(),
             old_q.w(), old_q.x(), old_q.y(), old_q.z(),
             new_p.x(), new_p.y(), new_p.z(),
             new_v.x(), new_v.y(), new_v.z(), new_v.norm(),
             new_ba.norm(), new_bw.norm(), new_g.norm(),
             new_q.w(), new_q.x(), new_q.y(), new_q.z(),
             has_wheel ? "YES" : "NO",
             wh_x, wh_y, wh_yaw * 180.0 / M_PI,
             cfg.ikfom.covariance.initial_cov * P_RELOC_GAIN,
             imu_stamp_used);
  }

};


int main(int argc, char** argv) {

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  
  ros::init(argc, argv, "limoncello");
  ros::NodeHandle nh("~");

  // Setup config parameters.
  Config& cfg = Config::getInstance();
  fill_config(cfg, nh);

  if (!lookup_tf_extrinsics(cfg)) {
    ROS_ERROR("TF extrinsics lookup FAILED! Using YAML fallback: t=[%.3f, %.3f, %.3f]",
              cfg.sensors.extrinsics.imu2baselink.translation().x(),
              cfg.sensors.extrinsics.imu2baselink.translation().y(),
              cfg.sensors.extrinsics.imu2baselink.translation().z());
  }

  // Initialize manager (reads from config)
  Manager manager = Manager(nh);

  // Subscribers
  ros::Subscriber lidar_sub = nh.subscribe(cfg.topics.input.lidar,
                                           1,
                                           &Manager::lidar_callback,
                                           &manager,
                                           ros::TransportHints().tcpNoDelay());

  ros::Subscriber imu_sub = nh.subscribe(cfg.topics.input.imu,
                                         1000,
                                         &Manager::imu_callback,
                                         &manager,
                                         ros::TransportHints().tcpNoDelay());

  ros::Subscriber stop_sub = nh.subscribe(cfg.topics.input.stop_ioctree_update,
                                          10,
                                          &Manager::stop_update_callback,
                                          &manager);


  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  ros::waitForShutdown();

  return 0;
}

