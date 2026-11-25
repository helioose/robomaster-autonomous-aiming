#include "lessons.hpp"
#include "io_context.hpp"
#include "lesson04_demo.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }

// ---------------- 第四讲验收演示 ----------------
//
// 画面里的框：
//   绿   = 真值（仿真器 /lesson/armor_truth 直接投影）
//   黄   = PnP（第三讲检测角点 + SolvePnP，按真实板尺寸重建后投影）
//   品红 = 预测（跟踪器外推 n 毫秒后的位置，沿用当前板面朝向）
//   橙   = 已经到达“预测目标时刻”的真值
//
// 静止靶：每帧显示 PnP 位置与真值位置的距离。
// 平移靶：每帧把“t+n 的预测”存档，真值里出现 t+n 那一帧时结算误差；
//         结算结果显示在左下角，直到下一次结算。

using Clock = training::Clock;

constexpr std::size_t kMaxPendingPredictions = 600;
const cv::Scalar kColorTruth(0, 255, 0);
const cv::Scalar kColorPnp(0, 200, 255);
const cv::Scalar kColorPrediction(255, 0, 255);
const cv::Scalar kColorChecked(0, 165, 255);
const cv::Scalar kColorHud(235, 235, 235);

struct TimedTruth {
  training::TimePoint stamp;
  std::vector<training::TruthArmor> armors;
};

struct LockedTarget {
  bool valid = false;
  std::uint32_t id = 0;
  bool large_armor = false;
  std::size_t truth_index = 0;  // 锁的是哪一块板（真值数组下标，帧间稳定）
  Eigen::Vector3d position_cam = Eigen::Vector3d::Zero();  // PnP 结果，相机光学系
  Eigen::Matrix3d rotation_cam = Eigen::Matrix3d::Identity();
  double pnp_error_m = 0.0;         // PnP 中心 vs 真值中心的距离
  Eigen::Vector3d pnp_error_cam = Eigen::Vector3d::Zero();  // 逐轴差值
  double reprojection_px = 0.0;
  const training::TruthArmor *truth = nullptr;
  training::Armor armor;    // 本帧与该目标对应的检测，只把它喂给跟踪器
  bool observed = false;    // 本帧是否真的检测到并更新了这块板
};

struct PendingPrediction {
  std::uint32_t id = 0;
  bool large_armor = false;
  std::size_t truth_index = 0;  // 预测的是哪一块板，结算时取同一块真值
  Eigen::Vector3d position_imu = Eigen::Vector3d::Zero();  // 预测的 t+n 位置
  Clock::time_point target_time{};
};

struct CheckedPrediction {
  bool valid = false;
  double target_ms = 0.0;
  double error_m = 0.0;
  std::size_t count = 0;
  double mean_error_m = 0.0;
  double max_error_m = 0.0;
};

std::string formatDouble(double value, int precision) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::vector<cv::Point2f> pixelCorners(const training::TruthArmor &armor) {
  // 真值消息的顺序是左上、左下、右上、右下；直接连多边形会画成蝴蝶结。
  // 换成左上、右上、右下、左下，和第三讲的角点顺序一致。
  return {armor.corners[0], armor.corners[2], armor.corners[3], armor.corners[1]};
}

// 检测角点与真值角点的像素距离：每块检测角点取最近的真值角点再平均。
// 同一块板约 1~2 px；侧面/背面等其他板会大得多。真值消息的角点顺序
// 与第三讲检测角点不完全一致，所以不做一一对应，只比较点集。
double cornerSetDistance(const training::Armor &armor,
                         const training::TruthArmor &truth) {
  double sum = 0.0;
  for (const cv::Point2f &corner : armor.corners) {
    double best = std::numeric_limits<double>::max();
    for (const cv::Point2f &truth_corner : truth.corners)
      best = std::min(best, cv::norm(corner - truth_corner));
    sum += best;
  }
  return sum / static_cast<double>(armor.corners.size());
}

// 真值里同一辆车会给出四块板（前、后、左、右）。只有灯条朝向相机的那块
// 才可能被检测到。用真值角点叉乘出板面法向，只保留法向大致指向相机的板，
// 避免把检测配到背面/侧面板上。
bool truthFacesCamera(const training::TruthArmor &truth) {
  const Eigen::Vector3d e1 = truth.corners_cam[1] - truth.corners_cam[0];
  const Eigen::Vector3d e2 = truth.corners_cam[2] - truth.corners_cam[0];
  Eigen::Vector3d normal = e1.cross(e2);
  if (normal.squaredNorm() < 1e-12)
    return false;
  normal.normalize();
  return normal.z() < -0.2;  // 相机光学系 z 向前，法向 z<0 才是朝向相机
}

// 预测位置放进相机系：IMU 稳定系 -> 云台系 -> 相机光学系。
Eigen::Vector3d imuToCamera(const Eigen::Vector3d &p_imu, const Eigen::Quaterniond &imu_q,
                            const training::CameraCalibration &calibration) {
  const Eigen::Vector3d p_gimbal = imu_q.inverse() * p_imu;
  return calibration.camera_to_gimbal.transpose() *
         (p_gimbal - calibration.camera_to_gimbal_translation);
}

// 真值从相机光学系放进 IMU 稳定系，与 imuToCamera 严格互逆。
// 与 lesson04_tracking.cpp 里 solveArmorPose 的链路一致：
//   p_gimbal = R_cam2gimbal * p_cam + t_cam2gimbal
//   p_imu    = imu_q * p_gimbal
Eigen::Vector3d cameraToImu(const Eigen::Vector3d &p_cam, const Eigen::Quaterniond &imu_q,
                            const training::CameraCalibration &calibration) {
  const Eigen::Vector3d p_gimbal =
      calibration.camera_to_gimbal * p_cam + calibration.camera_to_gimbal_translation;
  return imu_q * p_gimbal;
}

// 对一块检测做 PnP；返回相机光学系下的位姿与重投影 RMSE。
bool solveDetection(const training::Armor &armor,
                    const training::CameraCalibration &calibration,
                    Eigen::Vector3d &position_cam, Eigen::Matrix3d &rotation_cam,
                    double &reprojection_rmse) {
  // 第三讲给的是灯条外接矩形：仿真器实测约 140 mm × 59 mm。
  const double width = armor.large_armor ? 0.2386 : 0.140;
  const double height = 0.059;
  const std::vector<cv::Point3f> object_points = {
      cv::Point3f(static_cast<float>(-width / 2), static_cast<float>(-height / 2), 0.0f),
      cv::Point3f(static_cast<float>(width / 2), static_cast<float>(-height / 2), 0.0f),
      cv::Point3f(static_cast<float>(width / 2), static_cast<float>(height / 2), 0.0f),
      cv::Point3f(static_cast<float>(-width / 2), static_cast<float>(height / 2), 0.0f)};
  const std::vector<cv::Point2f> image_points(armor.corners.begin(), armor.corners.end());

  cv::Mat rvec;
  cv::Mat tvec;
  if (!cv::solvePnP(object_points, image_points, calibration.matrix, calibration.distortion,
                    rvec, tvec, false, cv::SOLVEPNP_IPPE))
    return false;

  cv::Mat rvec64;
  cv::Mat tvec64;
  rvec.convertTo(rvec64, CV_64F);
  tvec.convertTo(tvec64, CV_64F);
  position_cam = Eigen::Vector3d(tvec64.at<double>(0), tvec64.at<double>(1),
                                 tvec64.at<double>(2));
  if (!position_cam.allFinite() || position_cam.z() <= 0.1)
    return false;

  cv::Mat rotation;
  cv::Rodrigues(rvec64, rotation);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      rotation_cam(row, col) = rotation.at<double>(row, col);

  cv::Mat projected;
  cv::projectPoints(object_points, rvec, tvec, calibration.matrix, calibration.distortion,
                    projected);
  double squared = 0.0;
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f residual = projected.at<cv::Point2f>(i) - image_points[i];
    squared += static_cast<double>(residual.dot(residual));
  }
  reprojection_rmse = std::sqrt(squared / 4.0);
  return true;
}

// 真值中与给定中心最接近、且同 id 同尺寸的装甲板（在 IMU 系里比距离）。
const training::TruthArmor *nearestTruth(const std::vector<training::TruthArmor> &truth,
                                         std::uint32_t id, bool large_armor,
                                         const Eigen::Vector3d &position_imu,
                                         const Eigen::Quaterniond &imu_q,
                                         const training::CameraCalibration &calibration) {
  const training::TruthArmor *best = nullptr;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto &armor : truth) {
    if (armor.id != id || armor.large_armor != large_armor)
      continue;
    if (!truthFacesCamera(armor))
      continue;
    const double distance =
        (cameraToImu(armor.position_cam, imu_q, calibration) - position_imu).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best = &armor;
    }
  }
  return best;
}

int runDemo(training::io::IoContext &devices) {
  const auto &config = devices.config;
  const auto calibration = training::load_calibration(config);
  const double prediction_s = config.prediction_ms / 1000.0;
  const Clock::duration prediction_duration =
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(prediction_s));
  const Clock::duration kTruthHistoryAge = std::chrono::seconds(3);

  // 高 DPI 屏幕上 AUTOSIZE 会把 1440x1080 的图像放大到 2 倍，窗口比屏幕还高，
  // 左下角的误差就被切掉了。用可缩放窗口固定一个能完整显示的大小。
  if (config.display) {
    cv::namedWindow("lesson04 demo", cv::WINDOW_NORMAL);
    cv::resizeWindow("lesson04 demo", 960, 720);
  }

  cv::VideoWriter writer;
  bool writer_opened = false;
  std::vector<cv::Mat> pending_frames;      // 估算帧率期间先缓存
  std::vector<training::TimePoint> frame_stamps;

  training::TrackingState state;
  std::deque<TimedTruth> truth_history;
  std::deque<PendingPrediction> pending;
  CheckedPrediction checked;
  LockedTarget locked;

  std::size_t frames = 0;
  std::size_t detections_total = 0;
  std::size_t settled = 0;
  std::size_t pnp_samples = 0;
  double pnp_sum = 0.0;
  double pnp_smoothed = -1.0;

  auto openWriterIfNeeded = [&](const cv::Size &size) {
    if (!config.record)
      return true;
    if (writer_opened || pending_frames.size() < 20)
      return true;
    // 用缓存帧的时间戳中位数估计真实帧率，MP4 播放速度才不会失真。
    std::vector<double> intervals;
    intervals.reserve(frame_stamps.size() - 1);
    for (std::size_t i = 1; i < frame_stamps.size(); ++i)
      intervals.push_back(std::chrono::duration<double>(frame_stamps[i] - frame_stamps[i - 1]).count());
    std::sort(intervals.begin(), intervals.end());
    const double median = intervals[intervals.size() / 2];
    const double fps = median > 1e-6 ? 1.0 / median : 30.0;
    writer.open(config.record_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, size);
    if (!writer.isOpened()) {
      std::cerr << "无法打开录像文件：" << config.record_path << std::endl;
      return false;
    }
    std::cout << "开始录像：" << config.record_path << "（估计帧率 " << formatDouble(fps, 2)
              << " fps）" << std::endl;
    for (const auto &frame : pending_frames)
      writer.write(frame);
    pending_frames.clear();
    frame_stamps.clear();
    writer_opened = true;
    return true;
  };

  while (!interrupted) {
    const auto input = devices.read();
    if (!input) {
      if (cv::waitKey(1) == 27)
        break;
      continue;
    }
    const auto &data = *input;
    ++frames;

    truth_history.push_back({data.timestamp, data.truth});
    while (!truth_history.empty() &&
           data.timestamp - truth_history.front().stamp > kTruthHistoryAge)
      truth_history.pop_front();

    cv::Mat canvas = data.image.clone();

    // 真值框全部画出，方便看出检测漏了谁。
    for (const auto &truth : data.truth)
      training::drawArmorBox(canvas, pixelCorners(truth), kColorTruth, 2);

    // 1) 第三讲检测 + PnP，选一块与真值配上的板锁定。
    LockedTarget current;
    std::vector<training::Armor> armors;
    if (config.demo_pnp_source == "detection") {
      armors = training::lesson03_detection(data.image);
      detections_total += armors.size();
      // 真值按渲染帧发布，与图像 stamp 通常差 10~40 ms；目标在动时，
      // 同一块板的像素位置会差一二十像素。阈值要放宽，否则会误判“没配上”。
      double best_pixel_this_frame = 60.0;  // px，超过则认为与真值配不上
      double best_range = std::numeric_limits<double>::max();
      for (const auto &armor : armors) {
        Eigen::Vector3d position_cam;
        Eigen::Matrix3d rotation_cam;
        double reprojection = 0.0;
        if (!solveDetection(armor, calibration, position_cam, rotation_cam, reprojection))
          continue;
        // 用像素角点配真值板：同一块板的角点只差 1~2 px，侧面/背面板差很多。
        // 不用 PnP 位置配，是因为 PnP 本身有误差，配错板会把误差算到别的板上。
        const training::TruthArmor *truth = nullptr;
        std::size_t truth_index = 0;
        double best_corner_distance = 60.0;  // px
        for (std::size_t i = 0; i < data.truth.size(); ++i) {
          const training::TruthArmor &candidate_truth = data.truth[i];
          if (candidate_truth.large_armor != armor.large_armor)
            continue;
          if (!truthFacesCamera(candidate_truth))
            continue;
          const double distance = cornerSetDistance(armor, candidate_truth);
          if (distance < best_corner_distance) {
            best_corner_distance = distance;
            truth = &candidate_truth;
            truth_index = i;
          }
        }
        if (!truth)
          continue;

        // 已锁定：只刷新同一辆车的同一块板（真值数组下标帧间稳定）。
        // 同一辆车的四块板位置只差 0.2~0.4 m，只按 id 判断会在板面之间跳，
        // 预测框就会突然飘到车身侧面/背面去。
        // 未锁定：取距离最近的一块（导论里的“哪个近打哪个”），不看重投影误差——
        // 远处小板的平面 PnP 重投影误差可能反而更小，会把目标锁到别的车身上。
        const double range = position_cam.norm();
        const bool same_target =
            locked.valid && truth->id == locked.id && truth_index == locked.truth_index;
        const bool better = !locked.valid && range < best_range;
        if ((same_target || better) && best_corner_distance <= best_pixel_this_frame) {
          current.valid = true;
          current.id = truth->id;
          current.large_armor = truth->large_armor;
          current.truth_index = truth_index;
          current.position_cam = position_cam;
          current.rotation_cam = rotation_cam;
          current.reprojection_px = reprojection;
          current.truth = truth;
          current.armor = armor;
          current.observed = true;
          current.pnp_error_m = (position_cam - truth->position_cam).norm();
          current.pnp_error_cam = position_cam - truth->position_cam;
          best_pixel_this_frame = best_corner_distance;
          best_range = range;
        }
      }
    } else {
      // 调试路径：直接用真值中心做位姿，验证画框/坐标换算链。
      for (std::size_t i = 0; i < data.truth.size(); ++i) {
        const training::TruthArmor &truth = data.truth[i];
        if (locked.valid && truth.id != locked.id)
          continue;
        current.valid = true;
        current.id = truth.id;
        current.large_armor = truth.large_armor;
        current.truth_index = i;
        current.position_cam = truth.position_cam;
        current.rotation_cam = Eigen::Matrix3d::Identity();
        current.truth = &truth;
        current.pnp_error_m = 0.0;
        current.pnp_error_cam.setZero();
        break;
      }
    }
    if (current.valid) {
      locked = current;
      if (locked.pnp_error_m >= 0.0) {
        ++pnp_samples;
        pnp_sum += locked.pnp_error_m;
        pnp_smoothed = pnp_smoothed < 0 ? locked.pnp_error_m
                                        : 0.9 * pnp_smoothed + 0.1 * locked.pnp_error_m;
      }
    } else {
      locked.observed = false;  // 本帧没看到锁定的板：保留旧状态用于外推，但不重复喂观测
    }

    if (std::getenv("L4_DEBUG") && locked.valid && frames % 10 == 1) {
      const auto &t = *locked.truth;
      std::cerr << "DBG id=" << locked.id << " det=(" << armors.size() << ") pnp=("
                << locked.position_cam.transpose() << ") truth=("
                << t.position_cam.transpose() << ") d=" << locked.pnp_error_m
                << " reproj=" << locked.reprojection_px << std::endl;
      for (int i = 0; i < 4; ++i) {
        std::cerr << "  corner " << i << " det=(";
        if (!armors.empty())
          std::cerr << armors[0].corners[i];
        std::cerr << ") truth_px=(" << t.corners[i] << ") truth_cam=("
                  << t.corners_cam[i].transpose() << ")" << std::endl;
      }
    }

    // 2) 黄框：用 PnP 的位置和朝向投影一块真实尺寸的板。
    std::vector<cv::Point2f> pnp_px;
    if (locked.valid) {
      training::TruthArmor shape;
      shape.large_armor = locked.large_armor;
      if (projectArmorBox(shape, locked.rotation_cam, locked.position_cam, calibration, pnp_px))
        training::drawArmorBox(canvas, pnp_px, kColorPnp, 2);
    }

    // 3) 跟踪 + 外推：复用学员第四讲的卡尔曼跟踪函数。
    bool tracking = false;
    Eigen::Vector3d predicted_imu = Eigen::Vector3d::Zero();
    if (config.demo_pnp_source == "detection") {
      // 只把“锁定的那块板”喂给 lesson04_tracking：本演示验证的是预测，
      // 不应让选板阶段在开局短暂锁到画面里其他装甲板。
      std::vector<training::Armor> tracker_input;
      if (locked.valid && locked.observed)
        tracker_input.push_back(locked.armor);
      training::lesson04_tracking(tracker_input, data.timestamp, data.imu_q, calibration, state);
      tracking = state.initialized;
      // 连续多帧检测不到锁定的板（例如目标转走了）：放弃锁定，等下一块。
      if (!locked.observed && state.missed_frames >= 10)
        locked.valid = false;
    } else {
      tracking = state.initialized;
    }
    if (tracking) {
      // 跟踪器只在有观测时更新位置；中间丢检的这段时间也要按速度补上，
      // 保证预测永远是从“当前这一帧”往后推 n 毫秒。
      double since_last_update_s =
          std::chrono::duration<double>(data.timestamp - state.last_timestamp).count();
      since_last_update_s = std::clamp(since_last_update_s, 0.0, 0.5);
      predicted_imu =
          state.position + state.velocity * (since_last_update_s + prediction_s);
    }

    // 4) 品红框：t+n 时刻的预测位置（沿用当前板面朝向）。
    //    只在“本帧真的观测到了锁定的板”时才画并记入结算：
    //    盲推（换板/转走/丢检期间纯靠速度外推）的预测会明显偏，算进成绩不公平，
    //    也容易在两个位置各画一个品红框。
    if (tracking && locked.valid && locked.observed) {
      const Eigen::Vector3d predicted_cam = imuToCamera(predicted_imu, data.imu_q, calibration);
      std::vector<cv::Point2f> px;
      training::TruthArmor shape;
      shape.large_armor = locked.large_armor;
      if (projectArmorBox(shape, locked.rotation_cam, predicted_cam, calibration, px)) {
        // 静止靶时预测位置和 PnP 位置几乎重合，画出来只会盖住黄框；
        // 只有预测确实挪开了（平移靶）才画品红框并记入结算。
        double max_shift_px = 0.0;
        if (pnp_px.size() == px.size()) {
          for (std::size_t i = 0; i < px.size(); ++i)
            max_shift_px = std::max(max_shift_px, cv::norm(px[i] - pnp_px[i]));
        }
        if (max_shift_px > 3.0) {
          training::drawArmorBox(canvas, px, kColorPrediction, 2);

          PendingPrediction record;
          record.id = locked.id;
          record.large_armor = locked.large_armor;
          record.truth_index = locked.truth_index;
          record.position_imu = predicted_imu;
          record.target_time = data.timestamp + prediction_duration;
          pending.push_back(record);
          while (pending.size() > kMaxPendingPredictions)
            pending.pop_front();
        }
      }
    }

    // 5) 结算：真值时间戳到达 t+n 时，把预测位置和那一刻的真值比较。
    // 一帧最多只画一个橙框：帧间隔抖动或换板瞬间，可能有多条预测同时到期，
    // 逐条画会出现两个橙色真值框。只保留最后结算的那一条来画。
    const training::TruthArmor *orange_truth = nullptr;
    for (auto it = pending.begin(); it != pending.end();) {
      if (data.timestamp < it->target_time) {
        ++it;
        continue;
      }
      // 在已收到的真值快照里找采样时刻最接近 t+n 的一张。真值大约 19 Hz，
      // 若只用“本帧”的真值，平均会晚半个真值周期（约 25 ms）；
      // 目标 1 m/s 时这就是 2~3 cm 的系统误差。
      const TimedTruth *snapshot = nullptr;
      double best_offset_s = std::numeric_limits<double>::max();
      for (const auto &candidate : truth_history) {
        if (candidate.armors.empty())
          continue;
        const double offset_s =
            std::chrono::duration<double>(candidate.stamp - it->target_time).count();
        if (std::abs(offset_s) < std::abs(best_offset_s)) {
          best_offset_s = offset_s;
          snapshot = &candidate;
        }
      }
      // 附近 90 ms 内没有真值：中间掉帧太多，这一条不结算。
      if (!snapshot || std::abs(best_offset_s) > 0.09) {
        it = pending.erase(it);
        continue;
      }
      // 优先取预测时记住的那一块板（真值数组顺序帧间稳定）；
      // 真值条数/顺序对不上时，退回按预测位置找同 id 最近的真值。
      const training::TruthArmor *truth = nullptr;
      if (it->truth_index < snapshot->armors.size() &&
          snapshot->armors[it->truth_index].id == it->id)
        truth = &snapshot->armors[it->truth_index];
      else
        truth = nearestTruth(snapshot->armors, it->id, it->large_armor, it->position_imu,
                             data.imu_q, calibration);
      if (truth) {
        const Eigen::Vector3d truth_imu = cameraToImu(truth->position_cam, data.imu_q, calibration);
        const double error = (truth_imu - it->position_imu).norm();
        // 同 id 的装甲板可能属于远处的另一台车/建筑；差 1 m 以上视为配错目标，不结算。
        if (error > 1.0) {
          it = pending.erase(it);
          continue;
        }
        ++checked.count;
        const double previous_sum =
            checked.count > 1 ? checked.mean_error_m * static_cast<double>(checked.count - 1) : 0.0;
        checked.mean_error_m = (previous_sum + error) / static_cast<double>(checked.count);
        checked.max_error_m = std::max(checked.max_error_m, error);
        checked.error_m = error;
        checked.target_ms = config.prediction_ms;
        checked.valid = true;
        ++settled;
        orange_truth = truth;
      }
      it = pending.erase(it);
    }
    if (orange_truth)
      training::drawArmorBox(canvas, pixelCorners(*orange_truth), kColorChecked, 3);

    // 6) 左下角 HUD。
    //    演示窗口底部（约 16:9 播放裁切时会掉 80~135 px）可能看不到最后几行，
    //    所以把当前场景最关键的误差数据放最上面：
    //    静止靶先放速度/平均误差、|d|(ema)、逐轴差/重投影；
    //    平移靶一旦有结算结果，则把 TRANSLATE 的 |d|、mean/max 提到最上面。
    std::vector<std::string> lines;
    if (checked.valid) {
      lines.push_back("TRANSLATE  PREDICT(t+" + formatDouble(checked.target_ms, 0) +
                      "ms) vs TRUTH  |d|=" + formatDouble(checked.error_m, 3) + " m");
      lines.push_back("  mean=" + formatDouble(checked.mean_error_m, 3) +
                      " m  max=" + formatDouble(checked.max_error_m, 3) +
                      " m  checks=" + std::to_string(checked.count));
    }
    if (locked.valid) {
      lines.push_back("STATIC  speed=" + formatDouble(state.velocity.norm(), 2) +
                      " m/s  mean|d|=" +
                      (pnp_samples > 0 ? formatDouble(pnp_sum / static_cast<double>(pnp_samples), 3)
                                       : std::string("-")) +
                      " m");
      lines.push_back("STATIC  PnP vs TRUTH  |d|=" + formatDouble(locked.pnp_error_m, 3) +
                      " m  (ema " + formatDouble(pnp_smoothed, 3) + ")");
      lines.push_back("  cam d=(" + formatDouble(locked.pnp_error_cam.x(), 3) + ", " +
                      formatDouble(locked.pnp_error_cam.y(), 3) + ", " +
                      formatDouble(locked.pnp_error_cam.z(), 3) + ")  reproj=" +
                      formatDouble(locked.reprojection_px, 2) + " px");
    } else {
      lines.push_back("STATIC  PnP vs TRUTH  (no matched target)");
    }
    if (!checked.valid) {
      lines.push_back("TRANSLATE  waiting for t+" + formatDouble(config.prediction_ms, 0) +
                      "ms truth ...");
    }
    lines.push_back("PnP source: " + config.demo_pnp_source + "   detections: " +
                    std::to_string(detections_total));
    lines.push_back("green=TRUTH  yellow=PnP  magenta=PREDICT(t+" +
                    formatDouble(config.prediction_ms, 0) + "ms)  orange=TRUTH at t+n");
    training::drawHud(canvas, lines, kColorHud);

    // 7) 录像：前 20 帧先缓存，按估计帧率开写。
    if (config.record) {
      if (!writer_opened) {
        pending_frames.push_back(canvas.clone());
        frame_stamps.push_back(data.timestamp);
        if (!openWriterIfNeeded(canvas.size()))
          break;
      } else {
        writer.write(canvas);
      }
    }
    if (config.display) {
      cv::imshow("lesson04 demo", canvas);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q')
        break;
    }

    if (frames % 30 == 0) {
      std::cout << "frames=" << frames << " truth=" << data.truth.size()
                << " tracked=" << tracking << " pending=" << pending.size()
                << " checks=" << checked.count;
      if (locked.valid)
        std::cout << " pnp_err=" << formatDouble(locked.pnp_error_m, 4);
      if (checked.valid)
        std::cout << " pred_err=" << formatDouble(checked.error_m, 4);
      std::cout << std::endl;
    }
  }

  if (writer_opened)
    writer.release();
  else if (!pending_frames.empty() && config.record) {
    std::cerr << "录制帧数不足，未写出视频。" << std::endl;
  }
  std::cout << "\n演示结束：帧 " << frames << "，预测结算 " << settled << " 次";
  if (checked.count > 0)
    std::cout << "，平均误差 " << formatDouble(checked.mean_error_m, 3) << " m，最大 "
              << formatDouble(checked.max_error_m, 3) << " m";
  if (pnp_samples > 0)
    std::cout << "，PnP 平均误差 " << formatDouble(pnp_sum / static_cast<double>(pnp_samples), 3)
              << " m";
  std::cout << std::endl;
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  const auto config = training::load_config(argc > 1 ? argv[1] : "configs/simulation.yaml");
  const auto calibration = training::load_calibration(config);
  training::io::IoContext devices{config};
  training::TrackingState tracking;
  devices.open();

  if (config.demo) {
    const int code = runDemo(devices);
    devices.close();
    return code;
  }

  while (!interrupted) {
    const auto input = devices.read();
    training::Command command{};
    if (input) {
      const auto armors = training::lesson03_detection(input->image);
      const auto target = training::lesson04_tracking(
          armors, input->timestamp, input->imu_q, calibration, tracking);
      if (target)
        command = training::lesson05_aiming(*target, input->timestamp, input->bullet_speed);
    }
    if (interrupted) break;
    devices.write(command);

    if (input && devices.config.display) {
      cv::imshow("RoboMaster training", input->image);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q') break;
    }
  }

  devices.close();
}
