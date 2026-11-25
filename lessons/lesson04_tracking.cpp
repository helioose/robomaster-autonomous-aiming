#include "lessons.hpp"
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace training {
namespace {

// ---------- 装甲板物理尺寸（米） ----------
// 第三讲给的是“两根灯条外接矩形”的四个角，不是整块装甲板的外框。
// 仿真器里实测约 140 mm × 59 mm；按常见的 135 × 125 mm 解 PnP，
// 高度会被放大两倍多，距离会稳定偏近约 10 cm。
constexpr double kSmallArmorWidth = 0.140;
constexpr double kSmallArmorHeight = 0.059;
constexpr double kLargeArmorWidth = 0.2386;
constexpr double kLargeArmorHeight = 0.059;

// ---------- 跟踪器参数 ----------
constexpr double kMinConfidence = 0.40;          // 低于该置信度的装甲板不参与跟踪
constexpr double kMaxAssociateDistance = 1.0;    // m，观测与预测位置的最大关联距离
constexpr int kReinitMissedFrames = 5;           // 连续丢失多少帧后允许重新初始化
constexpr double kMaxFilterDt = 0.5;             // s，dt 过大说明中间丢帧，直接重置
constexpr double kMaxVelocity = 8.0;             // m/s，速度限幅

// ---------- 卡尔曼滤波参数 ----------
// 匀速（CV）模型：状态是 [位置, 速度]，过程噪声来自目标加速度。
// q 越大越相信观测（跟得快、抖动大），越小越相信模型（平滑、转弯滞后）。
constexpr double kProcessAccelNoise = 4.0;       // m/s²，目标加速度标准差
constexpr double kMeasurementNoise = 0.03;       // m，PnP 位置观测标准差
constexpr double kInitPositionVariance = 0.25;   // m²，初始位置方差
constexpr double kInitVelocityVariance = 4.0;    // (m/s)²，初始速度方差

// ---------- 新息门限 ----------
// 新息 ν = z − H·x'，新息协方差 S = H·P'·Hᵀ + R。
// d² = νᵀ·S⁻¹·ν 在模型自洽时服从 3 自由度 χ² 分布：
// 11.34 对应 99% 置信，超过就判为野值，本帧不更新。
constexpr double kGateChi2 = 11.34;

using KfCovariance = Eigen::Matrix<double, 6, 6>;

// 用一次观测直接重启滤波器（第一帧，或断档太久之后）。
void resetKalman(TrackingState &state, const Eigen::Vector3d &measurement) {
  state.position = measurement;
  state.velocity.setZero();
  state.covariance.setZero();
  state.covariance.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * kInitPositionVariance;
  state.covariance.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * kInitVelocityVariance;
  //协方差矩阵（代表不确定性）初始化，左上角3*3是位置方差，右下角3*3是速度方差，其他位置为位置-速度的协方差，初始为0
  state.initialized = true;
  state.missed_frames = 0;
}

// 预测步：x' = F x，P' = F P F^T + Q。
void predictKalman(TrackingState &state, double dt) {
  if (!(dt > 1e-6))
    return;

  KfCovariance F = KfCovariance::Identity();//单位矩阵
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  //右上角：表示旧速度影响新位置，位置是速度对时间的积分，所以是 dt·I
  //左下角：旧位置 → 新速度。匀速模型假设“位置不会让速度变化”，所以是 0
  state.position += state.velocity * dt;

  // 连续白噪声加速度模型离散化。
  const double q = kProcessAccelNoise * kProcessAccelNoise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  KfCovariance Q = KfCovariance::Zero();
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (q * dt3 / 3.0);
  Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (q * dt2 / 2.0);
  Q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (q * dt2 / 2.0);
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q * dt);

  state.covariance = F * state.covariance * F.transpose() + Q;
  //乘上F的转置是为了把协方差矩阵从上一时刻的状态空间映射到当前时刻的状态空间
}

// 更新步：H = [I 0]，用 PnP 位置修正位置和速度。
// 返回值 false 表示观测被新息门限判为野值，状态没有任何改动。
bool updateKalman(TrackingState &state, const Eigen::Vector3d &measurement) {
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  //H的作用是把状态向量映射到观测空间，这里只观测位置，所以H的左上角是单位矩阵，右上角是零矩阵
  const Eigen::Matrix3d R =
      Eigen::Matrix3d::Identity() * (kMeasurementNoise * kMeasurementNoise);
  //R表示观测噪声协方差矩阵，这里假设观测噪声是各向同性的，且标准差为kMeasurementNoise
  const Eigen::Matrix3d S = H * state.covariance * H.transpose() + R;
  //S是两个协方差表示的总不确定性，H * state.covariance * H.transpose()表示预测状态的不确定性，R表示观测噪声的不确定性

  // 新息门限：先用当前的 P' 和 R 判断这次观测离预测“离谱不离谱”。
  // 超门限时直接返回，不改位置/速度/协方差：保留预测，让协方差继续按 Q 增长，
  // 下一帧门限自然放宽 —— 对“越来越不确定”的状态就该越来越宽容。
  const Eigen::Vector3d innovation = measurement - state.position;
  const double mahalanobis_sq =
      innovation.transpose() * S.inverse() * innovation;
  if (!std::isfinite(mahalanobis_sq) || mahalanobis_sq > kGateChi2)
    return false;

  const Eigen::Matrix<double, 6, 3> K =
      state.covariance * H.transpose() * S.inverse();//inverse()是求矩阵的逆
  //K=P/(P+R)
  //K是卡尔曼增益矩阵，表示预测状态和观测之间的权重分配，K越大表示更相信观测，K越小表示更相信预测状态
  const Eigen::Matrix<double, 6, 1> correction = K * innovation;
  // correction是状态向量的修正量，表示预测状态和观测之间的差异，乘以卡尔曼增益矩阵得到对状态向量的修正
  state.position += correction.head<3>();
  state.velocity += correction.tail<3>();
  
  // Joseph 形式，数值上保持协方差对称正定。
  const KfCovariance IKH = KfCovariance::Identity() - K * H;
  state.covariance =
      IKH * state.covariance * IKH.transpose() + K * R * K.transpose();

  const double speed = state.velocity.norm();
  if (speed > kMaxVelocity)
    state.velocity *= kMaxVelocity / speed;
  return true;
}

// 一块装甲板完成 PnP 后的结果。
struct ArmorPose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // IMU 稳定系，m
  double reprojection_error = 0.0;                     // 像素 RMSE
};

// 装甲板平面上的四个角点，顺序与 Armor::corners 一致：
// 左上、右上、右下、左下。物体坐标系取 x 向右、y 向下、z 垂直板面朝外，
// 这样 PnP 解出来的就是相机光学坐标系（x 右、y 下、z 前）下的位姿。
std::vector<cv::Point3f> armorObjectPoints(bool large_armor) {
  const double half_width =
      (large_armor ? kLargeArmorWidth : kSmallArmorWidth) / 2.0;
  const double half_height =
      (large_armor ? kLargeArmorHeight : kSmallArmorHeight) / 2.0;
  return {
      cv::Point3f(static_cast<float>(-half_width),
                  static_cast<float>(-half_height), 0.0f),
      cv::Point3f(static_cast<float>(half_width),
                  static_cast<float>(-half_height), 0.0f),
      cv::Point3f(static_cast<float>(half_width),
                  static_cast<float>(half_height), 0.0f),
      cv::Point3f(static_cast<float>(-half_width),
                  static_cast<float>(half_height), 0.0f)};
}

double secondsBetween(TimePoint later, TimePoint earlier) {
  return std::chrono::duration<double>(later - earlier).count();
}

// 对单块装甲板做 PnP，并把位置一路换算到 IMU 稳定系。
//
//   p_camera --R_camera2gimbal, t--> p_gimbal --imu_q--> p_imu
//
// 只做“几何换算”，不在这里做跟踪滤波。
bool solveArmorPose(const Armor &armor, const CameraCalibration &calibration,
                    const Eigen::Quaterniond &imu_q, ArmorPose &pose) {
  const std::vector<cv::Point3f> object_points =
      armorObjectPoints(armor.large_armor);
  const std::vector<cv::Point2f> image_points(armor.corners.begin(),
                                              armor.corners.end());

  cv::Mat rvec;//旋转向量
  cv::Mat tvec;//平移向量
  // IPPE 专门解平面目标；装甲板是平面，4 个角点刚好够用。
  const bool solved = cv::solvePnP(object_points, image_points,
                                   calibration.matrix, calibration.distortion,
                                   rvec, tvec, false, cv::SOLVEPNP_IPPE);
  if (!solved) {
    return false;
  }

  // solvePnP 的输出深度可能跟随输入点，这里统一转成 double 再取值。
  cv::Mat rvec64;
  cv::Mat tvec64;
  rvec.convertTo(rvec64, CV_64F);
  tvec.convertTo(tvec64, CV_64F);
  if (rvec64.total() != 3 || tvec64.total() != 3) {
    return false;
  }

  const Eigen::Vector3d p_camera(tvec64.at<double>(0), tvec64.at<double>(1),
                                 tvec64.at<double>(2));
  // 光学坐标系 z 向前，z <= 0 说明解到了相机背后，属于无效解。
  if (!p_camera.allFinite() || p_camera.z() <= 0.1) {
    return false;
  }

  // 用投影回像素的误差衡量这次 PnP 是否可信。
  cv::Mat projected;
  cv::projectPoints(object_points, rvec, tvec, calibration.matrix,
                    calibration.distortion, projected);
  if (projected.total() != object_points.size()) {
    return false;
  }
  double squared_error = 0.0;
  for (std::size_t i = 0; i < object_points.size(); ++i) {
    const cv::Point2f residual = projected.at<cv::Point2f>(static_cast<int>(i)) -
                                 image_points[i];
    squared_error += static_cast<double>(residual.dot(residual));
  }
  pose.reprojection_error =
      std::sqrt(squared_error / static_cast<double>(object_points.size()));

  // 相机 -> 云台 -> IMU 稳定系。
  const Eigen::Vector3d p_gimbal =
      calibration.camera_to_gimbal * p_camera +
      calibration.camera_to_gimbal_translation;
  pose.position = imu_q * p_gimbal;
  return pose.position.allFinite();
}

// 在多个候选装甲板中选一个作为本帧的目标。
// 已经建立跟踪时，优先选择离“预测位置”最近的（保证前后帧盯的是同一块板）；
// 还没有跟踪时，选择重投影误差最小的（最可信的那块）。
int selectArmor(const std::vector<ArmorPose> &poses,
                const std::vector<Armor> &armors, TrackingState &state,
                TimePoint timestamp) {
  Eigen::Vector3d predicted = state.position;
  if (state.initialized) {
    const double dt = secondsBetween(timestamp, state.last_timestamp);
    if (dt > 0.0 && dt <= kMaxFilterDt) {
      predicted += state.velocity * dt;
    }
  }

  int best = -1;
  double best_score = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (armors[i].confidence < kMinConfidence) {
      continue;
    }
    double score = 0.0;
    if (state.initialized) {
      const double distance = (poses[i].position - predicted).norm();
      if (distance > kMaxAssociateDistance) {
        continue;  // 离预测太远，认为不是同一目标
      }
      score = distance;
    } else {
      score = poses[i].reprojection_error;
    }
    if (score < best_score) {
      best_score = score;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// 把当前（可能只是预测的）状态打包成输出。
Target toTarget(const TrackingState &state) {
  Target target;
  target.position = state.position;
  target.velocity = state.velocity;
  target.frames_since_update = state.frames_since_update;
  return target;
}

}  // namespace

// 第四讲：位姿、坐标与基础跟踪
//
// 输入格式：
//   armors     : const std::vector<Armor>&，第三讲输出的本帧装甲板观测。
//   timestamp  : TimePoint，本帧图像时刻；只能与其他 TimePoint 求时间差。
//   imu_q      : const Eigen::Quaterniond&，图像时刻的归一化姿态四元数。
//   calibration: const CameraCalibration&：
//                matrix=3x3 CV_64F 相机内参；distortion=1x5 CV_64F 畸变；
//                camera_to_gimbal=相机到云台的 3x3 旋转矩阵；
//                camera_to_gimbal_translation=相机到云台平移，单位 m。
//   state      : TrackingState&，仅本讲使用的跨帧状态；函数需要原地更新。
// 输出格式：
//   std::optional<Target>。有目标时：
//   - position : IMU 稳定参考系中、相对云台原点的目标向量，单位 m；
//   - velocity : 同一参考坐标系中的目标速度，单位 m/s；
//   - frames_since_update : 0 表示本帧有观测参与更新，非 0 表示纯预测
//                           （丢检、或本帧观测被新息门限判为野值）。
//   本课程统一使用 x 向前、y 向左、z 向上的右手坐标系。
//   跟踪已建立但本帧没有有效观测时，仍返回“预测值”并置 frames_since_update>0，
//   这样云台可以继续跟随，但第五讲据 frames_since_update 禁止开火。
//   跟踪尚未建立时返回 std::nullopt。
std::optional<Target> lesson04_tracking(
    const std::vector<Armor> &armors, TimePoint timestamp,
    const Eigen::Quaterniond &imu_q, const CameraCalibration &calibration,
    TrackingState &state) {
  if (!calibration.valid()) {
    ++state.missed_frames;
    return std::nullopt;
  }

  // 四元数理论上已归一化，这里再归一化一次，避免数值漂移放大位置误差。
  Eigen::Quaterniond q = imu_q;
  if (!q.coeffs().allFinite()) {
    ++state.missed_frames;
    return std::nullopt;
  }//检查是否存在NAN
  q.normalize();

  // 0) 预测步：无论本帧有没有检测到，都先把状态推进到当前时刻。
  //    这样短暂丢检时预测仍以“现在”为基准，而不是停在上一帧。
  if (state.initialized) {
    const double dt = secondsBetween(timestamp, state.last_timestamp);
    if (dt > kMaxFilterDt) {
      // 中间断档太久，旧速度不再可信，等下一次观测重新初始化。
      state.initialized = false;
      state.velocity.setZero();
      state.covariance = KfCovariance::Identity();
    } else if (dt > 1e-6) {
      predictKalman(state, dt);
    }
    state.last_timestamp = timestamp;
  }

  if (armors.empty()) {
    ++state.missed_frames;
    ++state.frames_since_update;
    return state.initialized ? std::optional<Target>(toTarget(state))
                             : std::nullopt;
  }

  // 1) 每块装甲板独立解 PnP，得到 IMU 稳定系中的候选位置。
  std::vector<ArmorPose> poses;
  std::vector<Armor> valid_armors;
  poses.reserve(armors.size());
  valid_armors.reserve(armors.size());
  for (const Armor &armor : armors) {
    ArmorPose pose;
    if (solveArmorPose(armor, calibration, q, pose)) {
      poses.push_back(pose);
      valid_armors.push_back(armor);
    }
  }
  if (poses.empty()) {
    ++state.missed_frames;
    ++state.frames_since_update;
    return state.initialized ? std::optional<Target>(toTarget(state))
                             : std::nullopt;
  }

  // 2) 选目标。
  int best = selectArmor(poses, valid_armors, state, timestamp);
  if (best < 0) {
    ++state.missed_frames;
    ++state.frames_since_update;
    // 长时间对不上，说明目标可能换车或跟踪已经跟丢；允许重新初始化。
    if (state.missed_frames < kReinitMissedFrames) {
      return state.initialized ? std::optional<Target>(toTarget(state))
                               : std::nullopt;
    }
    state.initialized = false;
    state.velocity.setZero();
    state.covariance = KfCovariance::Identity();
    best = 0;
    for (std::size_t i = 1; i < poses.size(); ++i) {
      if (poses[i].reprojection_error < poses[best].reprojection_error) {
        best = static_cast<int>(i);
      }
    }
  }

  // 3) 卡尔曼更新：第一帧（或刚重启）直接初始化，之后用观测修正。
  const Eigen::Vector3d measurement = poses[best].position;
  if (!state.initialized) {
    resetKalman(state, measurement);
    state.frames_since_update = 0;
  } else if (updateKalman(state, measurement)) {
    state.frames_since_update = 0;
  } else {
    // 野值：本帧观测没被采纳，状态仍是纯预测，交给第五讲判断能否开火。
    ++state.missed_frames;
    if (state.missed_frames >= kReinitMissedFrames) {
      // 连续多帧都过不了门限：不是单帧噪声，而是滤波器真的跟丢了
      // （目标急停/换板/状态发散）。此时接受这次观测重新起步，
      // 继续硬撑预测只会越飘越远。
      resetKalman(state, measurement);
      state.frames_since_update = 0;
    } else {
      ++state.frames_since_update;
      return std::optional<Target>(toTarget(state));
    }
  }

  state.last_timestamp = timestamp;
  state.missed_frames = 0;

  return toTarget(state);
}

}  // namespace training
