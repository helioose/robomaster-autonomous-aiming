#pragma once
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

namespace training {
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
constexpr double kPi = 3.14159265358979323846;

// IO 提供的算法输入；错误、连接状态和设备模式留在 IO 层。
struct CameraCalibration {
  cv::Mat matrix; // 3x3 CV_64F，相机光学坐标系
  cv::Mat distortion;
  Eigen::Matrix3d camera_to_gimbal = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_to_gimbal_translation = Eigen::Vector3d::Zero();
  unsigned width=0, height=0;
  bool valid() const { return !matrix.empty(); }
};

// 仿真器发布的装甲板真值（仅第四讲验收演示使用）。
struct TruthArmor {
  std::uint32_t id = 0;                    // 队伍+标签，帧间保持不变
  bool large_armor = false;
  std::array<cv::Point2f, 4> corners{};    // 像素，左上、右上、右下、左下
  std::array<Eigen::Vector3d, 4> corners_cam{};  // 相机光学系，m
  Eigen::Vector3d position_cam = Eigen::Vector3d::Zero();  // 装甲板中心，相机光学系，m
};

struct SensorData
{
  cv::Mat image;  // CV_8UC3，BGR，独立拥有像素内存
  TimePoint timestamp{};
  std::int64_t ros_stamp_ns = 0;  // 图像 header.stamp（ROS 系统时间，ns）
  Eigen::Quaterniond imu_q = Eigen::Quaterniond::Identity();// IMU 稳定系中相对云台原点的姿态
  double bullet_speed = 0;  // m/s
  std::vector<TruthArmor> truth;  // 与图像同帧的装甲板真值，未收到时为空
};

// 第三讲输出：一块装甲板的图像观测。
struct Armor {
  std::array<cv::Point2f, 4> corners{};  // 左上、右上、右下、左下
  bool large_armor = false;
  int class_id = -1;
  double confidence = 0;
};

// 第四讲输出：当前观测时刻的单目标状态。无目标用 std::nullopt 表示。
struct Target {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m，IMU稳定系中相对云台原点的目标向量
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s，同一坐标系
  // 输出时刻距最近一次“被观测修正”过了多少帧：0 表示本帧有观测参与更新。
  // 纯预测外推时该值会增大，第五讲据此决定是否允许开火。
  int frames_since_update = 0;
};

// 第五讲输出：绝对角使用 rad，IO 边界统一转换单位和方向。
struct Command {
  bool control = false;
  bool shoot = false;
  double yaw = 0;
  double pitch = 0;
};
}  // namespace training
