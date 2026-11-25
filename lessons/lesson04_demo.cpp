#include "lesson04_demo.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace training {
namespace {

// 与 lesson04_tracking.cpp 保持同一套装甲板尺寸（灯条外接矩形）。
constexpr double kSmallArmorWidth = 0.140;
constexpr double kSmallArmorHeight = 0.059;
constexpr double kLargeArmorWidth = 0.2386;
constexpr double kLargeArmorHeight = 0.059;

}  // namespace

bool projectArmorBox(const TruthArmor &armor, const Eigen::Matrix3d &rotation_cam,
                     const Eigen::Vector3d &position_cam,
                     const CameraCalibration &calibration,
                     std::vector<cv::Point2f> &corners_px) {
  // 板面法向取 PnP 旋转的 z 轴，再张成 x/y，得到板面内的两个方向。
  Eigen::Vector3d normal = rotation_cam.col(2);
  if (normal.squaredNorm() < 1e-9 || normal.z() < 0.0)
    normal = Eigen::Vector3d(0, 0, 1);
  normal.normalize();
  Eigen::Vector3d x_axis(1, 0, 0);
  x_axis -= normal * x_axis.dot(normal);
  if (x_axis.squaredNorm() < 1e-6)
    x_axis = Eigen::Vector3d(0, 1, 0) - normal * normal.y();
  x_axis.normalize();
  const Eigen::Vector3d y_axis = normal.cross(x_axis);

  const double half_width = (armor.large_armor ? kLargeArmorWidth : kSmallArmorWidth) / 2.0;
  const double half_height = (armor.large_armor ? kLargeArmorHeight : kSmallArmorHeight) / 2.0;
  const std::array<Eigen::Vector2d, 4> signs = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1),
                                                Eigen::Vector2d(1, 1), Eigen::Vector2d(-1, 1)};

  std::vector<cv::Point3f> obj;
  obj.reserve(4);
  for (const auto &sign : signs) {
    const Eigen::Vector3d p =
        position_cam + x_axis * (half_width * sign.x()) + y_axis * (half_height * sign.y());
    if (!(p.z() > 0.05))
      return false;
    obj.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                     static_cast<float>(p.z()));
  }

  // 点已经换算到相机系，直接以零位姿投影。
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  cv::Mat projected;
  cv::projectPoints(obj, rvec, tvec, calibration.matrix, calibration.distortion, projected);
  if (projected.total() != 4)
    return false;
  corners_px.clear();
  for (int i = 0; i < 4; ++i)
    corners_px.push_back(projected.at<cv::Point2f>(i));
  return true;
}

void drawArmorBox(cv::Mat &image, const std::vector<cv::Point2f> &corners_px,
                  const cv::Scalar &color, int thickness) {
  if (corners_px.size() != 4)
    return;
  std::vector<cv::Point> poly;
  poly.reserve(4);
  for (const cv::Point2f &point : corners_px)
    poly.emplace_back(cvRound(point.x), cvRound(point.y));
  cv::polylines(image, poly, true, color, thickness, cv::LINE_AA);
}

void drawHud(cv::Mat &image, const std::vector<std::string> &lines,
             const cv::Scalar &color) {
  constexpr double kScale = 0.85;
  constexpr int kThickness = 2;
  constexpr int kPad = 16;
  constexpr int kLineHeight = 32;

  int width = 0;
  for (const std::string &line : lines) {
    int baseline = 0;
    width = std::max(width,
                     cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, kScale, kThickness, &baseline)
                         .width);
  }
  const int height = kLineHeight * static_cast<int>(lines.size());
  const int x0 = kPad;
  const int y0 = image.rows - kPad - height + kLineHeight;

  // 半透明底板，保证文字在亮背景上也清楚。
  cv::Mat overlay = image.clone();
  cv::rectangle(overlay, cv::Rect(x0 - 10, y0 - 26, width + 20, height + 34), cv::Scalar(0, 0, 0),
                cv::FILLED);
  cv::addWeighted(overlay, 0.55, image, 0.45, 0.0, image);

  for (std::size_t i = 0; i < lines.size(); ++i) {
    cv::putText(image, lines[i], cv::Point(x0, y0 + kLineHeight * static_cast<int>(i)),
                cv::FONT_HERSHEY_SIMPLEX, kScale, color, kThickness, cv::LINE_AA);
  }
}

}  // namespace training
