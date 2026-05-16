#pragma once

#include <execution>
#include <numeric>
#include <algorithm>
#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Core/Imu.hpp"
#include "Core/Plane.hpp"
#include "Core/Octree.hpp"
#include <Core/S2.hpp>

#include "Utils/Config.hpp"
#include "Utils/PCL.hpp"


#include <manif/manif.h>
#include <manif/SGal3.h>
#include <manif/SE3.h>
#include <manif/Bundle.h>
#include <manif/Rn.h>


struct State {

  using BundleT = manif::Bundle<double,
      manif::SGal3,  // position & rotation & velocity & t
      manif::SE3,    // extrinsics
      manif::R3,     // angular bias
      manif::R3,     // acceleartion bias
      manif::R3      // gravity
  >;

  using Tangent = typename BundleT::Tangent; 
  
  template<int R = Eigen::Dynamic, int C = R>
  using Mat = Eigen::Matrix<double, R, C>;

  template<int N = Eigen::Dynamic>
  using Vec = Eigen::Matrix<double, N, 1>;


  static constexpr int DoF = BundleT::DoF;  // DoF whole state
  static constexpr int DoFS2 = DoF-1;       // DoF g as S2
  static constexpr int DoFNoise = 4*3;      // b_w, b_a, n_{b_w}, n_{b_a}
  static constexpr int DoFObs = manif::SGal3d::DoF + manif::SE3d::DoF;   // DoF obsevation equation

  BundleT X;
  Mat<DoFS2> P;
  Mat<DoFNoise> Q;

  Vec<3> w; // angular velocity (IMU input)
  Vec<3> a; // linear acceleration (IMU input)

  double stamp;

  State() : stamp(-1.0) { 
    Config& cfg = Config::getInstance();
    
    // Set initial state
    auto extrinsics = cfg.sensors.extrinsics;
    auto lidar2imu = extrinsics.imu2baselink.inverse() * extrinsics.lidar2baselink;
                                                                //                  Tangent (idx)
    X = BundleT(manif::SGal3d(extrinsics.imu2baselink.translation(),     //                    0
                              Eigen::Quaterniond(extrinsics.imu2baselink.linear()),         // 6
                              {0., 0., 0.},                     // vx, vy, vz                  3
                              0.),                              // delta t                     9
                manif::SE3d(lidar2imu),                         // isometry                   10
                manif::R3d(cfg.sensors.intrinsics.gyro_bias),   // b_w                        16
                manif::R3d(cfg.sensors.intrinsics.accel_bias),  // b_a                        19
                manif::R3d(Vec<3>::UnitZ()                      // g                          22
                           * extrinsics.gravity));

    P.setIdentity();
    P *= cfg.ikfom.covariance.initial_cov;

    w.setZero();
    a.setZero();

    // Control signal noise covariance (never changes)
    Q.setZero();
 
    Q.block<3, 3>(0, 0) = cfg.ikfom.covariance.gyro       * Eigen::Matrix3d::Identity(); // n_w
    Q.block<3, 3>(3, 3) = cfg.ikfom.covariance.accel      * Eigen::Matrix3d::Identity(); // n_a
    Q.block<3, 3>(6, 6) = cfg.ikfom.covariance.bias_gyro  * Eigen::Matrix3d::Identity(); // n_{b_w}
    Q.block<3, 3>(9, 9) = cfg.ikfom.covariance.bias_accel * Eigen::Matrix3d::Identity(); // n_{b_a}
  } 

  /**
   * 状态预测函数,根据 IMU 读数和时间增量预测状态的先验值和协方差。
   * 输入:
   * - imu: 当前时刻的 IMU 读数,包含线加速度和角速度,未去bias
   * - dt: 当前时刻与上次状态更新之间的时间增量,单位为秒
   * 输出:
   * - 无返回值,但会更新状态对象的 X 和 P 成员变量
   */
  void predict(const Imu& imu, const double& dt) {
PROFC_NODE("predict")

    Mat<DoF> Adj, Jr; // Adjoint_X(u)^{-1}, J_r(u)  Sola-18, [https://arxiv.org/abs/1812.01537]
    BundleT X_tmp = X.plus(f(imu.lin_accel, imu.ang_vel) * dt, Adj, Jr);
    
    // 重力g活在S2球面上(3D向量但模长固定), predict不改变g方向
    // 但协方差传播仍需要S2上正确的Adj/Jr, 所以用零增量算一下
      Mat<3> AdjS2, JrS2;
      S2::compose(g(), {0., 0., 0.}, AdjS2, JrS2);

      Adj.template bottomRightCorner<3, 3>() = AdjS2;
      Jr.template bottomRightCorner<3, 3>() = JrS2;

      // Left投影: 重力在状态向量中占3维(R3), 但S2切空间只有2维
      // Jx(2×3)把R3投影到S2切空间, 使得P从25维降到24维(DoFS2)
      Mat<2, 3> Jx;
      S2::ominus(g(), g(), Jx);

      Mat<DoFS2, DoF> left = Mat<DoFS2, DoF>::Identity();
      left.template bottomRightCorner<2, 3>() = Jx;
      
      // Right投影: Ju(3×2)从S2切空间映射回R3, 与left配对
      // left × (...) × right 实现 25维 → 24维 → 计算 → 24维结果
      Mat<3, 2> Ju;
      S2::oplus(g(), {0., 0.}, {}, Ju);

      Mat<DoF, DoFS2> right = Mat<DoF, DoFS2>::Identity();
      right.template bottomRightCorner<3, 2>() = Ju;

    Mat<DoFS2>           Fx = left * (Adj + Jr * df_dx(imu) * dt) * right; // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (26)
    Mat<DoFS2, DoFNoise> Fw = left * Jr * df_dw() * dt;                   // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (27)

    //协方差 P（"自信程度"的数字化) 随时间传播: 旧的不确定性经状态转移传播 + 新的噪声不确定性
    // 每次predict后P都会变大(越走越不确定), 直到update用观测缩小它
    P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose(); 

    X = X_tmp;

    // Save info
    a = imu.lin_accel;
    w = imu.ang_vel;

    stamp = imu.stamp;
  }


  void interpolate_to(const double& t) {
    double dt = t - this->stamp;
    assert(dt >= 0);

    X = X.plus(f(a, w) * dt);
  }

  /**
   * 构造连续时间动力学的切向量, 提供给 manif 库做 X.plus(f()*dt) 指数映射
   * 输入: IMU 原始读数 (lin_acc=加速度计, ang_vel=陀螺仪), 未去bias
   * 输出: SGal3 切向量 u, 描述状态"此刻的变化趋势"
   *
   * 函数内部做两件事:
   *   1) 从原始读数中去掉 bias 和重力, 还原真实物理量:
   *        真实角速度 = ang_vel - b_w        (陀螺读数 - 陀螺bias)
   *        真实加速度 = lin_acc - b_a - Rᵀg  (加速度读数 - 加速度bias - body系下的重力)
   *   2) 按 SGal3 切向量的排列格式打包:
   *        [0-2]  ρ = 0       位置分量置零, 因为 SGal3 指数映射会自动
   *                           从速度和加速度推算位置增量 (p += v·dt + ½a·dt²)
   *        [3-5]  真实加速度   → 速度变化率
   *        [6-8]  真实角速度   → 姿态变化率
   *        [9]    1           → 时间均匀流逝
   *
   *   bias/重力/外参 的切向量分量全为 0 (predict 不改它们, 留给 update 修正)
   */
  Tangent f(const Vec<3>& lin_acc, const Vec<3>& ang_vel) {

    Tangent u = Tangent::Zero();
    u.element<0>().coeffs() << 0., 0., 0., 
                               lin_acc - b_a() /* -n_a */ - R().transpose()*g(),
                               ang_vel - b_w() /* -n_w */,
                               1.;
    // u.element<3>().coeffs() = n_{b_w} 
    // u.element<4>().coeffs() = n_{b_a}
    
    return u;
  }

/**
 * 状态对状态的雅可比: 描述状态中每个分量的微小变化如何影响状态的整体变化趋势 f(u)
 * 输入: IMU 原始读数 (lin_acc=加速度计, ang_vel=陀螺仪), 未去bias
 * 输出: df_dx, 一个 DoF×DoF 的矩阵, 描述状态中每个分量的微小变化如何影响 f(u) 的每个分量 (即状态变化趋势)
 */
  Mat<DoF> df_dx(const Imu& imu) {
    Mat<DoF> out = Mat<DoF>::Zero();

    // velocity 
    out.block<3, 3>(3,  6) = -manif::skew(R().transpose()*g()); // w.r.t R := d(R^-1*g)/dR * d(R^-1)/dR
    out.block<3, 3>(3, 19) = -Mat<3>::Identity(); // w.r.t b_a 
    out.block<3, 3>(3, 22) = -R().transpose(); // w.r.t g
    // rotation
    out.block<3, 3>(6, 16) = -Mat<3>::Identity(); // w.r.t b_w

    return out;
  }

  // f()对噪声的偏导: 4种噪声源(陀螺/加速度/两个bias漂移)怎么影响状态
  // 乘以Q后 = 每次predict新增的不确定性(Fw·Q·Fwᵀ中的Fw)
  Mat<DoF, DoFNoise> df_dw() {
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Mat<DoF, DoFNoise> out = Mat<DoF, DoFNoise>::Zero();

    out.block<3, 3>( 6, 0) = -Mat<3>::Identity(); // w.r.t n_w
    out.block<3, 3>( 3, 3) = -Mat<3>::Identity(); // w.r.t n_a
    out.block<3, 3>(16, 6) =  Mat<3>::Identity(); // w.r.t n_{b_w}
    out.block<3, 3>(19, 9) =  Mat<3>::Identity(); // w.r.t n_{b_a}
    
    return out;
  }

  // 整体思路:
  //   predict让P变大了(不确定), 现在用LiDAR点云与地图匹配来"纠偏":
  //   1. h_model: 每个点→变换到全局→找地图中最近平面→
  //      算残差z(点到平面距离, 理想=0) 和雅可比H(状态怎么影响这个距离)
  //   2. 卡尔曼增益 K = f(P,H,R): 综合"我有多不确定(P)"和"观测多可靠(1/R)"
  //   3. 修正 X += K·z, 然后P变小(更确定了)
  //   4. 因为"找最近平面"依赖位姿, 修正后最近邻可能变了, 所以要迭代
  //      (这就是IESKF的"I"=Iterated)
  /**
   * 状态更新函数,根据 LiDAR 点云与地图的匹配结果修正状态的后验值和协方差。
   * 输入:
   * - cloud: 当前时刻的 LiDAR 点云, 已经转换到 IMU 坐标系下, 包含 N 个点
   * - map: 当前的地图, 以八叉树形式存储, 支持 KNN 查询, 用于找到每个点的最近平面
   * 输出:
   * - 无返回值, 但会更新状态对象的 X 和 P 成员变量
   */
  void update(PointCloudT::Ptr& cloud, charlie::Octree& map) {
PROFC_NODE("update")

    Config& cfg = Config::getInstance();

    if (map.size() == 0)
      return;

// 观测模型 h_model:
// 输入: 当前状态s  输出: 雅可比H(N×16)和残差z(N×1)
// 并行处理每个点: LiDAR局部坐标 → 全局坐标 → KNN找平面 → 算点到平面距离和雅可比
    auto h_model = [&](const State& s,
                       Mat<Eigen::Dynamic, DoFObs>& H,
                       Mat<Eigen::Dynamic, 1>&      z) {

      int N = cloud->size();

      std::vector<bool> chosen(N, false);
      Planes planes(N);

      std::vector<int> indices(N);
      std::iota(indices.begin(), indices.end(), 0);
      
      std::for_each(
        std::execution::par_unseq,
        indices.begin(),
        indices.end(),
        [&](int i) {
          PointT pt = cloud->points[i];
          Vec<3> p = pt.getVector3fMap().cast<double>();
          // 坐标链: p(LiDAR局部) → L2I(LiDAR→IMU) → isometry(IMU→全局)
          Vec<3> g = s.isometry() * s.L2I_isometry() * p; // global coords 

          std::vector<pcl::PointXYZ> neighbors;
          std::vector<float> pointSearchSqDis;
          map.knn(pcl::PointXYZ(g(0), g(1), g(2)),
                  cfg.ikfom.plane.points,
                  neighbors,
                  pointSearchSqDis);
          
          if (neighbors.size() < cfg.ikfom.plane.points 
              or pointSearchSqDis.back() > cfg.ikfom.plane.max_sqrt_dist)
                return;
          
          Eigen::Vector4d p_abcd = Eigen::Vector4d::Zero();
          if (not estimate_plane(p_abcd, neighbors, cfg.ikfom.plane.plane_threshold))
            return;
          
          chosen[i] = true;
          planes[i] = Plane(p, p_abcd);
        }
      ); // end for_each

      Planes valid_planes;

      for (int i = 0; i < N; i++) {
        if (chosen[i])
          valid_planes.push_back(planes[i]);        
      }

      H = Mat<>::Zero(valid_planes.size(), DoFObs);
      z = Mat<>::Zero(valid_planes.size(), 1);

      indices.resize(valid_planes.size());
      std::iota(indices.begin(), indices.end(), 0);

      // For each plane, calculate its derivative and distance
      std::for_each (
        std::execution::par_unseq,
        indices.begin(),
        indices.end(),
        [&](int i) {
          Plane m = valid_planes[i];

          // act()在变换点的同时输出雅可比J_s: "状态X微调时, 这个点的全局坐标怎么变"
          // H_i = 法向量ᵀ × J_s: 把3D移动投影到"垂直于平面"方向(只有这个方向影响距离)
          Mat<3, manif::SGal3d::DoF> J_s;
          Vec<3> g = s.X.element<0>().act(s.L2I_isometry() * m.p, J_s);

          H.block<1, manif::SGal3d::DoF>(i, 0) << m.n.head(3).transpose() * J_s;

          // Differentiate w.r.t. SE3
          if (cfg.ikfom.estimate_extrinsics) {
            Eigen::Matrix<double, 3, manif::SE3d::DoF> J_e;
            manif::SE3d(isometry() * L2I_isometry()).act(m.p, J_e);
            
            H.block<1, manif::SE3d::DoF>(i, manif::SGal3d::DoF) << m.n.head(3).transpose() * J_e;
          }

          // 残差 = 负的点到平面距离, 理想值为0; z≠0说明位姿有误差需要修正
          z(i) = -dist2plane(m.n, g);
        }
      );

    }; // end h_model

// IESKF迭代循环: h_model算H,z → 卡尔曼增益K → 修正X → 检查收敛 → 重复

    BundleT    X_predicted = X;     // 保存先验(predict后的状态), 迭代中用于计算dx
    Mat<DoFS2> P_predicted = P;

    Mat<Eigen::Dynamic, DoFObs> H;
    Mat<Eigen::Dynamic, 1>      z;
    Mat<DoFS2> KH;

    double R = cfg.ikfom.lidar_noise;  // 观测噪声: R越大→越不信LiDAR→修正越保守

    Vec<3> g_pred = X_predicted.element<4>().coeffs();

    int i(0);

    do {
      h_model(*this, H, z); // 用当前X计算H,z (每次迭代重算, 因为最近邻可能变了)

      // dx = 当前X与先验X_predicted的偏差(第一次迭代时=0)
      // 把P投影到当前迭代点的切空间(流形上不同点的切空间不同)
        Mat<DoF> J_;
        Vec<DoFS2> dx = X.minus(X_predicted, J_).coeffs().head(DoFS2);
        dx.tail(2) = S2::ominus(g(), g_pred);  // 重力用S2距离

        // J_inv: 切空间之间的坐标变换
        Mat<DoFS2> J_inv = J_.topLeftCorner(DoFS2, DoFS2).inverse();
        P = J_inv * P * J_inv.transpose(); // P投影到当前线性化点

      // 信息矩阵形式算K(数值更稳定):
      //   P_new⁻¹ = P⁻¹ + HᵀH/R   (先验信息 + 观测信息)
      //   K = P_new × Hᵀ/R
        Mat<DoFObs> HTH = H.transpose() * H / R;
        
        Mat<DoFS2>  P_inv = P.inverse();
        P_inv.template topLeftCorner<DoFObs, DoFObs>() += HTH;  // 叠加观测信息
        P_inv = P_inv.inverse();  // 取逆回到协方差

        Vec<DoFS2> Kz = P_inv.template topLeftCorner<DoFS2, DoFObs>() * H.transpose() * z / R;  // K·z

        KH.setZero();
        KH.template topLeftCorner<DoFS2, DoFObs>() = P_inv.template topLeftCorner<DoFS2, DoFObs>() * HTH;

      // 总修正量 = 观测驱动(Kz) + 先验约束((KH-I)·J⁻¹·dx)
      // 第一次迭代dx=0, 所以dx=Kz(纯观测); 后续两项都起作用
      dx = Kz + (KH - Mat<DoFS2>::Identity()) * J_inv * dx; 
      
      // 注入修正: 重力单独走S2球面, 其余走manif的plus(指数映射)
      Tangent tau = Tangent::Zero();
      tau.coeffs().head(DoF-3) = dx.head(DoF-3);

      // Update
      X = X.plus(tau);
      g(S2::oplus(g(), dx.tail(2)));

      if ((dx.array().abs() <= cfg.ikfom.tolerance).all())
        break;

    } while (i++ < cfg.ikfom.max_iters);

    // 协方差收缩: 观测带来了新信息, P变小, 我们更确定了
    P = (Mat<DoFS2>::Identity() - KH) * P;
    X = X;
  }


// Getters
  inline Vec<3>                p() const { return X.element<0>().translation();             }
  inline Mat<3>                R() const { return X.element<0>().quat().toRotationMatrix(); }
  inline Eigen::Quaterniond quat() const { return X.element<0>().quat();                    }
  inline Vec<3>                v() const { return X.element<0>().linearVelocity();          }
  inline double                t() const { return X.element<0>().t();                       }
  inline Vec<3>              b_w() const { return X.element<2>().coeffs();                  }
  inline Vec<3>              b_a() const { return X.element<3>().coeffs();                  }
  inline Vec<3>                g() const { return X.element<4>().coeffs();                  }

  //IMU坐标系 → 全局坐标系 的变换
  inline Eigen::Isometry3d isometry() const {
    Eigen::Isometry3d T;
    T.linear() = R();
    T.translation() = p();
    return T;
  }

  // LiDAR坐标系 → IMU坐标系 的变换
  inline Eigen::Isometry3d L2I_isometry() const {
    return X.element<1>().isometry();
  }

// Setters
  void p   (const Vec<3>& in)             { X.element<0>() = manif::SGal3d(in, quat(), v(), t()); }
  void v   (const Vec<3>& in)             { X.element<0>() = manif::SGal3d(p(), quat(), in, t()); }
  void quat(const Eigen::Quaterniond& in) { X.element<0>() = manif::SGal3d(p(), in, v(), t()); } 
  void b_w (const Vec<3>& in)             { X.element<2>() = manif::R3d(in);                   }
  void b_a (const Vec<3>& in)             { X.element<3>() = manif::R3d(in);                   }
  void g   (const Vec<3>& in)             { X.element<4>() = manif::R3d(in);                   }

};

typedef boost::circular_buffer<State> States;
