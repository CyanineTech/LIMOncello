#pragma once  

#include <vector>
#include <algorithm>
#include <execution>

#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

#include "Core/State.hpp"
#include "Utils/PCL.hpp"
#include "Utils/Profiler.hpp"
#include "Utils/Config.hpp"

/**
 * LiDAR 点云处理相关函数,包括点云去畸变、滤波、下采样等.
 * 从状态环形缓冲区中截取 [start, end) 附近的一小段状态序列.
 *
 * 这里处理的不是点云本身,而是“用于点云去畸变(deskew)的状态轨迹”: 
 * - states 一般是按时间保存的 State 缓冲区(boost::circular_buffer)
 * - 该函数会筛出 start 到 end 之间的状态
 * - 另外会额外保留 1 个 start 之前的状态,便于后续插值
 *
 * 因此,它对点云预处理的作用是“准备去畸变所需的状态数据”,
 * 而不是直接修改点云点坐标。
 */
States filter_states(const States& states, const double& start, const double& end) {

  // 返回的 States 是一个新的环形缓冲区, 里面包含了 [start, end) 附近的状态数据,
  States out(1000); // Always initialize circular buffer !!

  for (const auto& state : states) {
    if (state.stamp >= end)
      continue;

    if (state.stamp >= start)
      out.push_front(state);
    
    if (state.stamp < start) {
      out.push_front(state);
      break;
    }
  }

  return out;
}

/**
 * 点云去畸变函数,根据每个点的时间戳在状态缓冲区里插值出对应的状态,并把点从当时的状态坐标系变换到当前状态坐标系。
 * 输入:
 * - cloud: 原始点云,每个点都包含一个时间戳(  通过 point_time_func() 获取)
 * - state: 当前状态,包含当前时刻的位姿和其他状态信息
 * - buffer: 状态缓冲区,包含了当前状态以及之前一段时间的状态数据,用于插值
 * - offset: 点云时间戳的全局偏移,用于修正点云时间戳与状态时间戳之间的系统误差
 * - sweep_time: 当前点云帧的时间戳,用于计算每个点的相对时间
 * 输出:
 * - 去畸变后的点云,每个点都被变换到当前状态坐标系下,以消除运动畸变。 
 */
PointCloudT::Ptr deskew(const PointCloudT::Ptr& cloud,
                        const State& state,
                        const States& buffer,
                        const double& offset,
                        const double& sweep_time) {
  
PROFC_NODE("deskew")

  auto binary_search = [&](const double& t) {
    if (buffer.empty()) return -1;
    if (t <= buffer.front().stamp) return 0;
    if (t >= buffer.back().stamp)  return (int)(buffer.size() - 1);

    int l = 0, r = buffer.size() - 1;
    while (l < r) {
        int m = l + (r - l + 1) / 2;
        if (buffer[m].stamp <= t)
          l = m;
        else
          r = m - 1;
    }
    return l;
  };


  PointTime point_time = point_time_func();

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());
  out->points.resize(cloud->points.size());

  std::vector<int> indices(cloud->points.size());
  std::iota(indices.begin(), indices.end(), 0);

  std::for_each(
    std::execution::par_unseq,
    indices.begin(),
    indices.end(),
    [&](int k) {
      int i_f = binary_search(point_time(cloud->points[k], sweep_time) + offset);

      State X0 = buffer[i_f];
      X0.interpolate_to(point_time(cloud->points[k], sweep_time) + offset);

      Eigen::Isometry3f T0 = (X0.isometry() * X0.L2I_isometry()).cast<float>();
      Eigen::Isometry3f TN = (state.isometry() * state.L2I_isometry()).cast<float>();

      Eigen::Vector3f p;  
      p << cloud->points[k].x, cloud->points[k].y, cloud->points[k].z;

      p = TN.inverse() * T0 * p;

      PointT pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      pt.intensity = cloud->points[k].intensity;

      out->points[k] = pt;
    }
  );

  return out;
}

/**
 * 点云滤波函数,根据配置的条件过滤掉一些点,比如距离过远的点,或者在某些区域内的点,以提高地图质量和匹配效率。
 * 输入:
 * - cloud: 输入点云,通常是已经去畸变后的点云
 * - lidar2baselink: LiDAR 到基座坐标系的变换,用于在滤波条件中把点云点从 LiDAR 坐标系转换到基座坐标系进行判断
 * 输出:
 * - 滤波后的点云,只包含满足条件的点,以提高后续处理的效率和地图质量。
 */
PointCloudT::Ptr filter(const PointCloudT::Ptr& cloud, 
                        const Eigen::Isometry3d& lidar2baselink) {

PROFC_NODE("filter")

  Config& cfg = Config::getInstance();

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());

  int index = 0;
  std::copy_if(
    cloud->points.begin(), 
    cloud->points.end(), 
    std::back_inserter(out->points), 
    [&](const PointT& p) mutable {
        bool pass = true;
        Eigen::Vector3f p_ = lidar2baselink.cast<float>() * p.getVector3fMap();

        // Distance filter
        if (cfg.filters.min_distance.active) {
          if (p_.squaredNorm() 
              <= cfg.filters.min_distance.value*cfg.filters.min_distance.value)
              pass = false;
        }

        // Crop box
        if (pass and cfg.filters.crop_box.active) {
          Eigen::Vector3f mn = cfg.filters.crop_box.min.cast<float>();
          Eigen::Vector3f mx = cfg.filters.crop_box.max.cast<float>();
          if ((p_.cwiseMax(mn).cwiseMin(mx).array() == p_.array()).all())
            pass = false;
        }

        // Rate filter
        if (pass and cfg.filters.rate_sampling.active) {
          if (index % cfg.filters.rate_sampling.value != 0)
              pass = false;
        }

        // Field of view filter
        if (pass and cfg.filters.fov.active) {
          if (fabs(atan2(p_.y(), p_.x())) >= cfg.filters.fov.value)
              pass = false;
        }

        ++index; // Increment index

        return pass;
    }
  );

  return out;
}

/**
 * 点云下采样函数,使用 PCL 的 VoxelGrid 滤波器对点云进行下采样。
 * 输入:
 * - cloud: 输入点云,通常是已经去畸变和滤波后的点云
 * 输出:
 * - 下采样后的点云,每个体素格子内的点被替换为一个代表点,以减少点云的数量并加快后续处理速度。
 */
PointCloudT::Ptr voxel_grid(const PointCloudT::Ptr& cloud) {

PROFC_NODE("downsample")

  Config& cfg = Config::getInstance();

  static pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(cfg.filters.voxel_grid.leaf_size.cast<float>());

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());
  filter.setInputCloud(cloud);
  filter.filter(*out);

  return out;
}
