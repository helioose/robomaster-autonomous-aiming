#pragma once
#include "types.hpp"
#include <array>
#include <string>
namespace training {
struct Config {
  bool display = true;
  bool allow_fire = false;
  std::array<double, 9> camera_matrix{
      1303.6753236814714, 0, 720, 0, 1303.6753236814714, 540, 0, 0, 1};
  std::array<double, 5> distort_coeffs{0, 0, 0, 0, 0};
  std::array<double, 9> camera_to_gimbal{
      0, 0.4226182617, 0.9063077870, -1, 0, 0,
      0, -0.9063077870, 0.4226182617};
  std::array<double, 3> camera_to_gimbal_translation{0, 0, 0};

  // 第四讲验收演示（tools 里的 lesson04 demo）：不进入正常的五讲流水线。
  bool demo = false;
  // PnP 用哪套角点："detection" 跑第三讲检测，"truth" 直接用仿真真值角点（调试用）。
  std::string demo_pnp_source = "detection";
  // 预测多少毫秒后的位置。
  double prediction_ms = 250.0;
  // 是否把叠加画面录成 MP4。
  bool record = false;
  std::string record_path = "lesson04_demo.mp4";
};
Config load_config(const std::string& path);
CameraCalibration load_calibration(const Config& config);
}
