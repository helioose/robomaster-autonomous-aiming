#pragma once
#include "types.hpp"
#include <vector>

namespace training {
// 第四讲验收演示用的小工具（只负责几何与画图，不读写 ROS/配置）。
//
// 三个概念：
//   真值框   —— 仿真器发布的装甲板角点，直接投影像素；
//   PnP 框   —— 检测角点 + SolvePnP 得到的位姿，按真实板尺寸重建 3D 角点后投影；
//   预测框   —— 把位置换成 t+n 的预测值、朝向沿用当前 PnP，再投影。
//
// 颜色约定写在 src/main.cpp 的演示循环里。

// 按真实装甲板尺寸重建 4 个 3D 角点（相机光学系）再投影像素。
// rotation_cam 只取第三列作为板面法向；位置可以是 PnP 结果，也可以是预测值。
bool projectArmorBox(const TruthArmor& armor, const Eigen::Matrix3d& rotation_cam,
                     const Eigen::Vector3d& position_cam,
                     const CameraCalibration& calibration,
                     std::vector<cv::Point2f>& corners_px);

void drawArmorBox(cv::Mat& image, const std::vector<cv::Point2f>& corners_px,
                  const cv::Scalar& color, int thickness = 2);

// 画面左下角多行文字。lines 自上而下排列，底板半透明，保证录像里也清楚。
void drawHud(cv::Mat& image, const std::vector<std::string>& lines,
             const cv::Scalar& color);
}  // namespace training
