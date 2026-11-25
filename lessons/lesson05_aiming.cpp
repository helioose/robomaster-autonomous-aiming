#include "lessons.hpp"

#include <chrono>
#include <cmath>
#include <optional>

namespace training
{
  namespace
  {

    // ===== 第 1 步：常量 =====
    constexpr double KGravity = 9.81;// m/s²，重力加速度
    constexpr double KPitchZero = 25.0 * kPi / 180.0;// 初始仰角，单位 rad
    constexpr double KMaxPitch = 45.0 * kPi / 180.0;// 最大仰角，单位 rad
    constexpr double KMinRange = 0.5;// m，低于该距离无法解算弹道
    constexpr double KMinSpeed = 1.0;// m/s，低于该速度无法解算弹道
  } // namespace
  struct BallisticSolution
  {
    double elevation = 0.0;// 弹道仰角，单位 rad
    double flight_time = 0;// 弹丸飞行时间，单位 s
  };
  std::optional<BallisticSolution> solveBallistic(double v0, double d,
                                                    double h)
  {
    if (!(v0 > KMinSpeed) || !std::isfinite(v0))//isfinite()函数用于检查一个数值是否为有限数值（即不是INF或NaN)
      return std::nullopt;
    if (!(d > KMinRange) || !std::isfinite(d) || !std::isfinite(h))
      return std::nullopt;
    const double a = 0.5 * KGravity * d * d / (v0 * v0);
    const double discriminant = d * d - 4.0 * a * (a + h); // 判别式
    if (discriminant < 0.0)
      return std::nullopt;
    const double tan_theta = (d - std::sqrt(discriminant)) / (2.0 * a);
    const double theta = std::atan(tan_theta);
    const double flight_time = d / (v0 * std::cos(theta));
    if (!std::isfinite(theta) || !std::isfinite(flight_time))
      return std::nullopt;
    return BallisticSolution{theta, flight_time};
  }
  // 第五讲：弹道解算
  //
  // 输入格式：
  //   target       : const Target&；position 单位 m，velocity 单位 m/s，二者均采用
  //                  第四讲规定的 x 向前、y 向左、z 向上坐标系。
  //   timestamp    : TimePoint，target 对应的图像时刻，用于计算预测时间。
  //   bullet_speed : double，弹丸初速度，单位 m/s，必须为有限正数。
  // 输出格式：
  //   Command：
  //   - control : true 表示请求云台跟随；无法产生安全解时必须为 false；
  //   - shoot   : true 表示请求发射，最终仍受 IO 层 allow_fire 和有效性门控；
  //   - yaw     : 世界/IMU 参考系下的绝对偏航角，单位 rad；
  //   - pitch   : 世界/IMU 参考系下的绝对俯仰角，单位 rad。
  // 验收：仿真器完成静止靶、平移靶任务。
  Command lesson05_aiming(const Target &target, TimePoint timestamp, double bullet_speed)
  {
    (void)target;
    (void)timestamp;
    (void)bullet_speed;
    return Command{};
  }
} // namespace training
