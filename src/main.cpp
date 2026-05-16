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
      ROS_DEBUG_THROTTLE(2.0, "[LIMONCELLO] Propagate waiting: buffer=%.6f end_scan=%.6f gap=%.3fms",
                         state_buffer_.front().stamp, end_stamp,
                         (end_stamp - state_buffer_.front().stamp) * 1000.0);

      std::unique_lock<decltype(mtx_buffer_)> lock(mtx_buffer_);
      // 等待状态数据准备好。
      cv_prop_stamp_.wait(lock, [this, &end_stamp] { 
          return state_buffer_.front().stamp >= end_stamp;
      });
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
  // 完整动作:
  //   1. 用 wheel odom 的当前位姿重置 state (p, quat), 速度清零
  //      → 保证从一个合理的物理位置重建地图
  //      → 防止发散后 state 仍在百万级 → 地图建在百万米外 → 永远无法恢复
  //   2. 清空 iOctree 地图
  //   3. 大幅放大 P → IESKF 前几帧几乎完全信任 LiDAR 观测, 快速收敛
  //   4. 清空 state_buffer_
  //   5. bias/g 保留不动 (硬件特性不变, P 放大后 IESKF 会重新估计)
  void initialpose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    Config& cfg = Config::getInstance();

    // 记录重置前的 state (用于诊断日志)
    Eigen::Vector3d old_p, old_v;
    Eigen::Quaterniond old_q;
    {
      std::lock_guard<std::mutex> lk(mtx_state_);
      old_p = state_.p();
      old_v = state_.v();
      old_q = state_.quat();
    }

    // 从 wheel odom 获取重置目标位姿
    bool has_wheel = false;
    double wh_x = 0, wh_y = 0, wh_yaw = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
      if (wheel_odom_received_) {
        has_wheel = true;
        wh_x = latest_wheel_odom_.pose.pose.position.x;
        wh_y = latest_wheel_odom_.pose.pose.position.y;
        const auto& oq = latest_wheel_odom_.pose.pose.orientation;
        wh_yaw = std::atan2(2.0*(oq.w*oq.z + oq.x*oq.y),
                            1.0 - 2.0*(oq.y*oq.y + oq.z*oq.z));
      }
    }

    mtx_state_.lock();

      if (has_wheel) {
        // 用 wheel odom 的 2D 位姿重置 state (Z 保持 0, roll/pitch 保持不变)
        Eigen::Vector3d new_p(wh_x, wh_y, 0.0);
        // imu2baselink 逆变换: state 存的是 IMU 帧位姿, 需要 T_M_I = T_M_B * T_B_I
        Eigen::Isometry3d T_B_I = cfg.sensors.extrinsics.imu2baselink;
        Eigen::Isometry3d T_M_B;
        T_M_B.setIdentity();
        T_M_B.translation() = new_p;
        T_M_B.linear() = Eigen::AngleAxisd(wh_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Isometry3d T_M_I = T_M_B * T_B_I;

        state_.p(T_M_I.translation());
        state_.quat(Eigen::Quaterniond(T_M_I.linear()).normalized());
        state_.v(Eigen::Vector3d::Zero());
      } else {
        // wheel odom 还没收到, 只能把速度清零, 位置不动
        state_.v(Eigen::Vector3d::Zero());
        ROS_WARN("[LIMOncello] /initialpose: no wheel odom yet, "
                 "state position NOT reset (only v=0)");
      }

      // 大幅放大 P, 让 IESKF 快速收敛到 LiDAR 观测
      state_.P.setIdentity();
      state_.P *= cfg.ikfom.covariance.initial_cov * 1000.0;

      ioctree_.clear();

    mtx_state_.unlock();

    mtx_buffer_.lock();
      state_buffer_.clear();
    mtx_buffer_.unlock();

    // 诊断日志: 重置前后对比 + wheel odom 值, 方便定位发散→恢复是否成功
    Eigen::Vector3d new_p_log, new_v_log;
    Eigen::Quaterniond new_q_log;
    {
      std::lock_guard<std::mutex> lk(mtx_state_);
      new_p_log = state_.p();
      new_v_log = state_.v();
      new_q_log = state_.quat();
    }

    ROS_WARN("[LIMOncello] /initialpose received -> state reset + map cleared\n"
             "  BEFORE: p=[%.3f, %.3f, %.3f] v=[%.3f, %.3f, %.3f] "
             "q=[w=%.4f, xyz=(%.4f, %.4f, %.4f)]\n"
             "  AFTER:  p=[%.3f, %.3f, %.3f] v=[%.3f, %.3f, %.3f] "
             "q=[w=%.4f, xyz=(%.4f, %.4f, %.4f)]\n"
             "  wheel_odom: %s (x=%.3f, y=%.3f, yaw=%.3f deg)\n"
             "  P *= %.1f, ioctree cleared, buffer flushed. Reconverging...",
             old_p.x(), old_p.y(), old_p.z(),
             old_v.x(), old_v.y(), old_v.z(),
             old_q.w(), old_q.x(), old_q.y(), old_q.z(),
             new_p_log.x(), new_p_log.y(), new_p_log.z(),
             new_v_log.x(), new_v_log.y(), new_v_log.z(),
             new_q_log.w(), new_q_log.x(), new_q_log.y(), new_q_log.z(),
             has_wheel ? "YES" : "NO",
             wh_x, wh_y, wh_yaw * 180.0 / M_PI,
             cfg.ikfom.covariance.initial_cov * 1000.0);
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

