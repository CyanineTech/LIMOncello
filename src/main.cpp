#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
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
  std::mutex mtx_map_;
  std::mutex mtx_wheel_odom_;

  std::condition_variable cv_prop_stamp_;

  charlie::Octree ioctree_;
  bool stop_ioctree_update_;
  bool reset_imu_integration_;
  std::atomic<uint64_t> reset_generation_;
  double reset_wheel_odom_timeout_;

  nav_msgs::Odometry latest_wheel_odom_;
  bool wheel_odom_received_;
  Eigen::Vector3d reset_gyro_bias_;
  Eigen::Vector3d reset_accel_bias_;

  ros::Publisher pub_state_, 
                 pub_frame_,
                 pub_raw_, 
                 pub_deskewed_, 
                 pub_downsampled_, 
                 pub_filtered_,
                 reset_done_pub_;

  ros::Subscriber reset_sub_;
  ros::Subscriber wheel_odom_sub_;

  tf2_ros::TransformBroadcaster br;

  static double yawFromQuat(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

public:
  Manager(ros::NodeHandle& nh) : first_imu_stamp_(-1.0), 
                                 state_buffer_(1000), 
                                 stop_ioctree_update_(false),
                                 reset_imu_integration_(false),
                                 reset_generation_(0),
                                 reset_wheel_odom_timeout_(0.5),
                                 wheel_odom_received_(false),
                                 ioctree_() {

    Config& cfg = Config::getInstance();
    reset_gyro_bias_ = cfg.sensors.intrinsics.gyro_bias;
    reset_accel_bias_ = cfg.sensors.intrinsics.accel_bias;

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

    reset_done_pub_ = nh.advertise<std_msgs::Header>("/limoncello/reset_done", 10);
    nh.param("reset_wheel_odom_timeout", reset_wheel_odom_timeout_, 0.5);
    reset_sub_ = nh.subscribe("/limoncello/reset_pose", 10,
        &Manager::reset_callback, this);
    wheel_odom_sub_ = nh.subscribe("/odom_wheel", 50,
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

        reset_gyro_bias_ = state_.b_w();
        reset_accel_bias_ = state_.b_a();

        prev_imu_ = imu;  // 初始化 prev_imu_，避免下一帧 dt 计算异常
        imu_calibrated_ = true;
      }

    } else {
      State state_snapshot;
      bool reset_sample = false;
      double dt = imu.stamp - prev_imu_.stamp;

      if (dt < 0)
        ROS_ERROR("IMU timestamps not correct");

      //时间戳检查: 如果 IMU 时间不连续或跳变过大,可能导致状态预测异常,这里简单地限制 dt 的范围,避免跳到未来或过度预测
      dt = (dt < 0 or dt >= imu.stamp) ? 1./cfg.sensors.imu.hz : dt;

      // Correct acceleration
      //校正加速度计的系统误差:这里只是用简单的比例因子做的矫正
      imu.lin_accel = cfg.sensors.intrinsics.sm * imu.lin_accel;

      //用校准后的 IMU 数据预测状态,并将预测结果存入缓冲区,供 LiDAR 回调使用 (位置,速度,姿态等)
      mtx_state_.lock();
      if (reset_imu_integration_) {
        prev_imu_ = imu;
        state_.stamp = imu.stamp;
        reset_imu_integration_ = false;
        reset_sample = true;
      } else {
        prev_imu_ = imu;
        state_.predict(imu, dt);
      }
        state_snapshot = state_;
      mtx_state_.unlock();

      mtx_buffer_.lock();
        state_buffer_.push_front(state_snapshot);
      mtx_buffer_.unlock();

      cv_prop_stamp_.notify_one();

      // 以 IMU 频率(~200Hz)发布里程计和 TF, 是系统的高频位姿输出
      if (reset_sample)
        ROS_INFO("[LIMO][reset] IMU integration restarted at %.6f", imu.stamp);
      pub_state_.publish(toROS(state_snapshot, imu.stamp));
      if (Config::getInstance().publish_tf)
        publishTFs(state_snapshot, br, imu.stamp);
    }
  }


  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    Config& cfg = Config::getInstance();
    const uint64_t generation_at_start = reset_generation_.load();

    //检查IMU数据是否完成校准,如果没有校准完成,则不处理 LiDAR 数据,避免使用不可靠的状态预测结果
    if (not imu_calibrated_)
      return;
    
    //检查是否收到处理后的IMU数据
    {
      std::lock_guard<std::mutex> lk(mtx_buffer_);
      if (state_buffer_.empty()) {
        ROS_ERROR("[LIMONCELLO] No IMUs received");
        return;
      }
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
      double state_stamp = 0.0;
      {
        std::lock_guard<std::mutex> lk(mtx_state_);
        state_stamp = state_.stamp;
      }
      offset = state_stamp - point_time(raw->points.back(), sweep_time) - 1.e-4;
      if (offset > 0.0) offset = 0.0; // don't jump into future
    }

    // 计算当前整帧点云的起止时间:
    // 后面会用这个时间区间去状态缓冲区里做插值/去畸变。
    double start_stamp = point_time(raw->points.front(), sweep_time) + offset;
    double end_stamp   = point_time(raw->points.back(), sweep_time) + offset;

    // 如果状态缓冲区里最新可用状态的时间 still 早于本帧点云结束时间,
    // 说明 IMU/传播线程还没有把状态推到这帧扫描结束位置,
    // 此时先阻塞等待,直到 buffer 覆盖到 end_stamp 为止。
    double latest_buffer_stamp = 0.0;
    {
      std::lock_guard<std::mutex> lk(mtx_buffer_);
      latest_buffer_stamp = state_buffer_.front().stamp;
    }
    if (latest_buffer_stamp < end_stamp) {
      double initial_gap_ms = (end_stamp - latest_buffer_stamp) * 1000.0;
      ROS_DEBUG_THROTTLE(2.0, "[LIMONCELLO] Propagate waiting: buffer=%.6f end_scan=%.6f gap=%.3fms",
                         latest_buffer_stamp, end_stamp, initial_gap_ms);

      auto t_wait_start = ros::WallTime::now();
      std::unique_lock<decltype(mtx_buffer_)> lock(mtx_buffer_);
      // 等待状态数据准备好。
      cv_prop_stamp_.wait(lock, [this, &end_stamp] { 
          return !state_buffer_.empty() && state_buffer_.front().stamp >= end_stamp;
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
    if (interpolated.size() == 0 or start_stamp < interpolated.front().stamp) {
      // every point needs to have a state associated not in the past
      ROS_WARN("Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

    if (generation_at_start != reset_generation_.load())
      return;

    State state_after_update;
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
    {
      std::lock_guard<std::mutex> lk_map(mtx_map_);
      state_.update(filtered, ioctree_);
    }

    Eigen::Isometry3f T = (state_.isometry() * state_.L2I_isometry()).cast<float>();
    state_after_update = state_;

  mtx_state_.unlock();

    PointCloudT::Ptr global(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*deskewed, *global, T);

    PointCloudT::Ptr to_save(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*filtered, *to_save, T);

    // Publish
    if (generation_at_start != reset_generation_.load())
      return;

    pub_state_.publish(toROS(state_after_update, sweep_time));
    pub_frame_.publish(toROS(global, sweep_time));

    if (cfg.debug) {
      pub_raw_.publish(toROS(raw, sweep_time));
      pub_deskewed_.publish(toROS(deskewed, sweep_time));
      pub_downsampled_.publish(toROS(downsampled, sweep_time));
      pub_filtered_.publish(toROS(to_save, sweep_time));
    }

    // Update map: 将当前帧的点云（已经去畸变、滤波、变换到全局坐标系）加入地图的八叉树结构中, 以供后续帧的匹配使用。
    if (not stop_ioctree_update_ && generation_at_start == reset_generation_.load()) {
      std::lock_guard<std::mutex> lk_map(mtx_map_);
      ioctree_.update(to_save->points);
    }

    if (cfg.verbose)
      PROFC_PRINT()
  }

  //保存最近一次收到的 /odom_wheel 消息, 用于在 /limoncello/reset_pose 时获取车轮里程计的位姿信息
  void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
    latest_wheel_odom_ = *msg;
    wheel_odom_received_ = true;
  }


  void reset_callback(const std_msgs::Header::ConstPtr& msg) {
    Config& cfg = Config::getInstance();
    const ros::Time request_stamp = msg->stamp.isZero() ? ros::Time::now() : msg->stamp;

    bool wheel_seen = false;
    bool has_wheel = false;
    double wheel_x = 0.0, wheel_y = 0.0, wheel_yaw = 0.0;
    double wheel_age = -1.0, wheel_vx = 0.0, wheel_wz = 0.0;
    {
      std::lock_guard<std::mutex> lk(mtx_wheel_odom_);
      //读取最近一次收到的 /odom_wheel 消息
      if (wheel_odom_received_) {
        wheel_seen = true;
        wheel_x = latest_wheel_odom_.pose.pose.position.x;
        wheel_y = latest_wheel_odom_.pose.pose.position.y;
        wheel_yaw = yawFromQuat(latest_wheel_odom_.pose.pose.orientation);
        wheel_vx = latest_wheel_odom_.twist.twist.linear.x;
        wheel_wz = latest_wheel_odom_.twist.twist.angular.z;
        //检查 /odom_wheel 的时间戳是否过期, 如果过期则认为没有可用的车轮里程计数据
        if (latest_wheel_odom_.header.stamp.isZero()) {
          wheel_age = 0.0;
          has_wheel = true;
        } else {
          wheel_age = (ros::Time::now() - latest_wheel_odom_.header.stamp).toSec();
          has_wheel = std::abs(wheel_age) <= reset_wheel_odom_timeout_;
        }
      }
    }

    const char* wheel_status = has_wheel ? "FRESH" : (wheel_seen ? "STALE" : "NO");
    ROS_WARN("[LIMO][reset] request stamp=%.6f wheel=%s age=%.3fs timeout=%.3fs pose=(%.3f %.3f %.2fdeg) vx=%.3f wz=%.3f",
             request_stamp.toSec(), wheel_status, wheel_age, reset_wheel_odom_timeout_,
             wheel_x, wheel_y, wheel_yaw * 180.0 / M_PI, wheel_vx, wheel_wz);

    Eigen::Vector3d old_p, old_v, new_p, new_v;
    Eigen::Quaterniond old_q, new_q;
    size_t old_map_size = 0;

    std_msgs::Header done;
    done.stamp = request_stamp;
    done.frame_id = has_wheel ? "ok" : (wheel_seen ? "stale_wheel_pose" : "no_wheel_pose");

    {
      std::lock(mtx_state_, mtx_buffer_, mtx_map_);
      std::lock_guard<std::mutex> lk_state(mtx_state_, std::adopt_lock);
      std::lock_guard<std::mutex> lk_buffer(mtx_buffer_, std::adopt_lock);
      std::lock_guard<std::mutex> lk_map(mtx_map_, std::adopt_lock);

      old_p = state_.p();
      old_v = state_.v();
      old_q = state_.quat();
      old_map_size = ioctree_.size();

      if (has_wheel) {
        Eigen::Isometry3d T_O_B = Eigen::Isometry3d::Identity();
        T_O_B.translation() = Eigen::Vector3d(wheel_x, wheel_y, 0.0);
        T_O_B.linear() = Eigen::AngleAxisd(wheel_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // State stores odom_limoncello -> IMU. The config matrix is base_link -> IMU.
        Eigen::Isometry3d T_O_I = T_O_B * cfg.sensors.extrinsics.imu2baselink;
        state_.p(T_O_I.translation());
        state_.quat(Eigen::Quaterniond(T_O_I.linear()).normalized());
      } else {
        ROS_WARN("[LIMO][reset] no fresh /odom_wheel; keeping current position and only resetting velocity/map/filter state");
      }

      state_.v(Eigen::Vector3d::Zero());
      state_.b_w(reset_gyro_bias_);
      state_.b_a(reset_accel_bias_);
      state_.g(Eigen::Vector3d::UnitZ() * cfg.sensors.extrinsics.gravity);
      state_.P.setIdentity();
      state_.P *= cfg.ikfom.covariance.initial_cov * 100.0;
      state_.stamp = request_stamp.toSec();

      ioctree_.clear();
      state_buffer_.clear();
      state_buffer_.push_front(state_);
      reset_imu_integration_ = true;
      reset_generation_.fetch_add(1);

      new_p = state_.p();
      new_v = state_.v();
      new_q = state_.quat();
    }

    cv_prop_stamp_.notify_all();
    reset_done_pub_.publish(done);

    // 打印重置前后的状态信息, 方便调试和验证
    ROS_WARN("[LIMO][reset] done stamp=%.6f map_points=%zu->0\n"
             "  before p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] q=[%.4f %.4f %.4f %.4f]\n"
             "  after  p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] q=[%.4f %.4f %.4f %.4f]",
             request_stamp.toSec(), old_map_size,
             old_p.x(), old_p.y(), old_p.z(), old_v.x(), old_v.y(), old_v.z(),
             old_q.w(), old_q.x(), old_q.y(), old_q.z(),
             new_p.x(), new_p.y(), new_p.z(), new_v.x(), new_v.y(), new_v.z(),
             new_q.w(), new_q.x(), new_q.y(), new_q.z());
  }


  void stop_update_callback(const std_msgs::Bool::ConstPtr& msg) {
    if (not stop_ioctree_update_ and msg->data) {
      stop_ioctree_update_ = msg->data;
      ROS_INFO("Stopping ioctree updates from now onwards");
    }
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
