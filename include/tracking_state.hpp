#pragma once
#include "types.hpp"

namespace training {
// 仅第四讲使用；由学员设计跨帧状态，其余讲不接收它。
//
// 匀速模型卡尔曼滤波：状态 = [位置(3), 速度(3)]，单位 m 和 m/s，
// 都在 IMU 稳定系里。协方差用来表示“这次估计有多可信”，
// 有观测时用观测修正，没有观测时只靠状态转移外推。
struct TrackingState {
  bool initialized = false;                       // 是否已经建立跟踪
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m，IMU 稳定系
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s，IMU 稳定系
  TimePoint last_timestamp{};                     // 上一次有效更新的图像时刻
  int missed_frames = 0;                          // 连续未观测到的帧数
  int frames_since_update = 0;                    // 距最近一次观测更新的帧数
  // 状态协方差：[位置 xyz, 速度 xyz] 的顺序，6x6。
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Identity();
};
}  // namespace training
