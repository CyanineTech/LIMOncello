#pragma once

#include <boost/make_shared.hpp>

#include <functional>
#include <iostream>
#include <algorithm>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "Utils/Config.hpp"


struct EIGEN_ALIGN16 PointT {
  PCL_ADD_POINT4D;
  float intensity;
  union {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint32_t, t, t)
  (float, time, time)
  (double, timestamp, timestamp)
)

typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::function<double(const PointT&, const double&)> PointTime;
typedef std::function<bool(const PointT&, const PointT&)> PointTimeComp;

/**
 * LiDAR 时间戳处理函数: 根据配置的 LiDAR 类型和时间戳定义,返回一个用于计算点云中每个点的时间戳的函数.
 * 该函数接受一个点和当前扫描的时间戳,根据 LiDAR 类型和时间戳定义计算出该点的绝对时间戳,用于后续的点云去畸变和状态更新.
 * 支持的 LiDAR 类型包括 OUSTER, VELODYNE, HESAI, LIVOX, 每种类型的时间戳定义不同,需要根据配置进行处理.
 * 该函数使得代码能够适配不同类型的 LiDAR 传感器,并且能够正确处理它们的时间戳,保证点云数据的准确性和一致性.
 */
PointTime point_time_func() {
  Config& cfg = Config::getInstance();

  if (cfg.sensors.lidar.type == 0) { // OUSTER
    return cfg.sensors.lidar.end_of_sweep
      ? [] (const PointT& p, const double& sweep_time) { return sweep_time - p.t * 1e-9; }
      : [] (const PointT& p, const double& sweep_time) { return sweep_time + p.t * 1e-9; };

  } else if (cfg.sensors.lidar.type == 1) { // VELODYNE
    return cfg.sensors.lidar.end_of_sweep
      ? [] (const PointT& p, const double& sweep_time) { return sweep_time - (double)p.time; }
      : [] (const PointT& p, const double& sweep_time) { return sweep_time + (double)p.time; };

  } else if (cfg.sensors.lidar.type == 2) { // HESAI
    return [] (const PointT& p, const double& sweep_time) { return p.timestamp; };

  } else if (cfg.sensors.lidar.type == 3) { // LIVOX
    return [] (const PointT& p, const double& sweep_time) { return p.timestamp * 1e-9; };

  } else {
    std::cout << "-------------------------------------------\n";
    std::cout << "LiDAR sensor type unknown or not specified!\n";
    std::cout << "-------------------------------------------\n";
    throw std::runtime_error("LiDAR sensor type unknown or not specified");
  }
}

/**
 * LiDAR 点云时间戳比较函数: 根据配置的 LiDAR 类型和时间戳定义,返回一个用于比较点云中两个点的时间戳的函数.
 * 该函数接受两个点,根据 LiDAR 类型和时间戳定义比较它们的时间戳,返回一个布尔值表示第一个点是否比第二个点更早或更晚.
 */
PointTimeComp get_point_time_comp() {
  Config& cfg = Config::getInstance();

  PointTimeComp point_time_cmp;

  if (cfg.sensors.lidar.type == 0) {
    
    if (cfg.sensors.lidar.end_of_sweep)
      point_time_cmp = [](const PointT& p1, const PointT& p2) { return p1.t > p2.t; };
    else
      point_time_cmp = [](const PointT& p1, const PointT& p2) { return p1.t < p2.t; };
  
  } else if (cfg.sensors.lidar.type == 1) {

    if (cfg.sensors.lidar.end_of_sweep)
      point_time_cmp = [](const PointT& p1, const PointT& p2) { return p1.time > p2.time; };
    else
      point_time_cmp = [](const PointT& p1, const PointT& p2) { return p1.time < p2.time; };

  } else if (cfg.sensors.lidar.type == 2 or cfg.sensors.lidar.type == 3) {
    point_time_cmp = [](const PointT& p1, const PointT& p2) { return p1.timestamp < p2.timestamp; };

  } else {
    std::cout << "-------------------------------------------\n";
    std::cout << "LiDAR sensor type unknown or not specified!\n";
    std::cout << "-------------------------------------------\n";
    throw std::runtime_error("LiDAR sensor type unknown or not specified");
  }

  return point_time_cmp;
}