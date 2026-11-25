#include "lessons.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
namespace training
{
  namespace
  {
    // 灯条：最小外接矩形 + 长边与竖直方向的夹角（单位：度）
    struct LightBar
    {
      cv::RotatedRect rect;
      float angle = 0.0f;
    };

    struct CandidatePair
    {
      size_t first;
      size_t second;
      float score;
      float angleDiff;
      float relativeLengthDiff;
      float relativeDy;
      float relativeDx;
    };

    std::array<cv::Point2f, 4> getOrderedCorners(const cv::RotatedRect &rect)
    {
      cv::Point2f points[4];
      rect.points(points);
      std::array<cv::Point2f, 4> corners = {
          points[0], points[1], points[2], points[3]};

      std::sort(corners.begin(), corners.end(),
                [](const cv::Point2f &a, const cv::Point2f &b)
                {
                  if (a.y != b.y)
                    return a.y < b.y;
                  return a.x < b.x;
                });

      std::sort(corners.begin(), corners.begin() + 2,
                [](const cv::Point2f &a, const cv::Point2f &b)
                { return a.x < b.x; });
      std::sort(corners.begin() + 2, corners.end(),
                [](const cv::Point2f &a, const cv::Point2f &b)
                { return a.x < b.x; });
      return corners;
    }

    cv::Mat createBlueMask(const cv::Mat &image)
    {
      cv::Mat hsv;
      cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
      cv::Mat blue_mask;
      cv::Scalar lower_blue(90, 100, 100);
      cv::Scalar upper_blue(124, 255, 255);
      cv::inRange(hsv, lower_blue, upper_blue, blue_mask);
      cv::GaussianBlur(blue_mask, blue_mask, cv::Size(3, 3), 0);
      cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
      return blue_mask;
    }

    std::vector<LightBar> findLightBars(const cv::Mat &blue_mask, cv::Mat &draw_contours)
    {
      std::vector<std::vector<cv::Point>> contours;
      cv::Mat contours_input = blue_mask.clone();
      cv::findContours(contours_input,
                       contours,
                       cv::RETR_EXTERNAL,        // 外部轮廓
                       cv::CHAIN_APPROX_SIMPLE); // 只保留轮廓的端点信息
      std::vector<LightBar> light_bars;

      for (size_t i = 0; i < contours.size(); i++) // 遍历轮廓，初步筛选出极不合理的灯条
      {
        double area = cv::contourArea(contours[i]);
        if (area < 9.0f)
          continue;
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);
        const float length = std::max(rect.size.width, rect.size.height);
        const float width = std::min(rect.size.width, rect.size.height);
        if (width <= 0.0f)
          continue;
        const float ratio = length / width;
        if (ratio < 3.0f)
          continue;
        if (length >= 500.0f)
          continue;
        if (width > 200.0f)
          continue;
        cv::Point2f corners[4];
        rect.points(corners);
        const cv::Point2f edge0 = corners[1] - corners[0];
        const cv::Point2f edge1 = corners[2] - corners[1]; // 用point2f表示向量
        auto longAxis = cv::norm(edge0) > cv::norm(edge1) ? edge0 : edge1;

        if (longAxis.y < 0.0f)
        {
          longAxis.x *= -1;
          longAxis.y *= -1;
        } // 保证长轴向量的y分量为正
        const float angle = std::atan2(longAxis.x, longAxis.y) * 180.0f / CV_PI;
        // 带正负号的角度，顺时针为正，逆时针为负
        if (std::abs(angle) > 20.0f)
          continue;
        cv::drawContours(draw_contours, contours, i, cv::Scalar(0, 0, 255), 2);
        light_bars.push_back({rect, angle}); // 将符合条件的灯条加入列表，angle后面会用来判断灯条是否平行
      }

      std::sort(light_bars.begin(), light_bars.end(), [](const LightBar &a, const LightBar &b)
                { return a.rect.center.y < b.rect.center.y; });

      for (size_t k = 0; k < light_bars.size(); ++k)
      {
        cv::putText(draw_contours,
                    std::to_string(k),
                    light_bars[k].rect.center,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(255, 255, 0),
                    2);
      }
      return light_bars;
    }

    std::vector<CandidatePair> findCandidatePairs(const std::vector<LightBar> &light_bars)
    {
      std::vector<CandidatePair> candidates;

      // 第一层：硬性筛选，直接淘汰明显不合理的灯条组合。
      for (size_t i = 0; i < light_bars.size(); ++i)
      {
        for (size_t j = i + 1; j < light_bars.size(); ++j)
        {
          const float angleDiff = std::abs(light_bars[i].angle - light_bars[j].angle);
          if (angleDiff > 1.8f)
            continue;

          const float dy = std::abs(light_bars[i].rect.center.y - light_bars[j].rect.center.y);
          const float lengthA = std::max(light_bars[i].rect.size.width,
                                         light_bars[i].rect.size.height);
          const float lengthB = std::max(light_bars[j].rect.size.width,
                                         light_bars[j].rect.size.height);
          const float lengthSum = lengthA + lengthB;
          if (lengthSum <= 0.0f)
            continue;

          const float relativeLengthDiff = std::abs(lengthA - lengthB) / lengthSum;
          if (relativeLengthDiff > 0.7f)
            continue;

          const float averageLength = lengthSum / 2.0f;
          const float relativeDy = dy / averageLength;
          if (relativeDy > 0.4f)
            continue;

          const float dx = std::abs(light_bars[i].rect.center.x - light_bars[j].rect.center.x);
          const float relativeDx = dx / averageLength;
          if (relativeDx < 1.0f || relativeDx > 3.0f)
            continue;

          // 第二层：归一化各项偏差后加权评分，权重占比可调整。
          constexpr float angleWeight = 0.30f;
          constexpr float lengthWeight = 0.25f;
          constexpr float heightWeight = 0.25f;
          constexpr float spacingWeight = 0.20f;

          // 与理想值差值归一化成0~1之间的惩罚值，惩罚值越大，评分越低
          const float anglePenalty = angleDiff / 1.8f;
          const float lengthPenalty = relativeLengthDiff / 0.7f;
          const float heightPenalty = relativeDy / 0.4f;
          const float spacingPenalty = std::abs(relativeDx - 2.0f);
          const float score = 100.0f * (1.0f -
                                        (angleWeight * anglePenalty +
                                         lengthWeight * lengthPenalty +
                                         heightWeight * heightPenalty +
                                         spacingWeight * spacingPenalty));
          candidates.push_back({i, j, score, angleDiff, relativeLengthDiff, relativeDy, relativeDx});
        }
      }
      return candidates;
    }

    // 最终配对
    std::vector<Armor> drawMatchedPairs(cv::Mat &draw_contours,
                                        const std::vector<LightBar> &light_bars,
                                        std::vector<CandidatePair> candidates,
                                        std::vector<Armor> Armors)
    {
      // 优先选择评分最高的候选；已配对的灯条不再参与后续配对。
      std::sort(candidates.begin(), candidates.end(),
                [](const CandidatePair &a, const CandidatePair &b)
                {
                  if (a.score != b.score)
                    return a.score > b.score;
                  if (a.first != b.first)
                    return a.first < b.first;
                  return a.second < b.second;
                });

      std::vector<bool> matched(light_bars.size(), false);
      constexpr float minPairScore = 60.0f;
      for (const CandidatePair &candidate : candidates)
      {
        if (candidate.score < minPairScore)
          continue;
        if (matched[candidate.first] || matched[candidate.second])
          continue;

        matched[candidate.first] = true;
        matched[candidate.second] = true;
        std::cout << "配对索引: "
                  << candidate.first << " - " << candidate.second
                  << " | score: " << candidate.score
                  << " | angleDiff: " << candidate.angleDiff
                  << " | relativeLengthDiff: " << candidate.relativeLengthDiff
                  << " | relativeDy: " << candidate.relativeDy
                  << " | relativeDx: " << candidate.relativeDx
                  << std::endl;

        cv::line(draw_contours,
                 light_bars[candidate.first].rect.center,
                 light_bars[candidate.second].rect.center,
                 cv::Scalar(0, 255, 0),
                 2);

        auto cornersA = getOrderedCorners(
            light_bars[candidate.first].rect); // 左上，右上，左下，右下

        auto cornersB = getOrderedCorners(
            light_bars[candidate.second].rect); // 左上，右上，左下，右下

        // 判断哪根灯条在左边
        const bool AisLeft =
            light_bars[candidate.first].rect.center.x <
            light_bars[candidate.second].rect.center.x;

        // 确定左右灯条
        const auto &left = AisLeft ? cornersA : cornersB;
        const auto &right = AisLeft ? cornersB : cornersA;

        // 创建装甲板
        Armor armor;

        // 左上、右上、右下、左下
        armor.corners = {
            left[0],
            right[1],
            right[3],
            left[2]};

        // 暂时无法识别编号
        armor.class_id = -1;

        // 几何配对置信度（防御性写法，score<0.0时返回0, score>100.0时返回1.0）
        armor.confidence = std::clamp(
            static_cast<double>(candidate.score) / 100.0,
            0.0,
            1.0);

        for (size_t i = 0; i < armor.corners.size(); ++i)
        {
          cv::line(draw_contours,
                   armor.corners[i],
                   armor.corners[(i + 1) % armor.corners.size()],
                   cv::Scalar(0, 255, 0),
                   2);
        }
        cv::putText(draw_contours,
                    cv::format("%.2f", armor.confidence), // format相当于stringstream
                    armor.corners[0],                     // 字体左下角锚点
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    2);

        Armors.push_back(armor);
      }
      return Armors;
    }
  } // namespace

  // 第三讲：装甲板检测
  // 输入：不可修改的 CV_8UC3 BGR 图像。
  // 输出：Armor 列表；角点顺序为左上、右上、右下、左下，单位为像素。
  // 图像坐标原点在左上；无法识别编号时 class_id=-1；没有装甲板时返回空列表。
  std::vector<Armor> lesson03_detection(const cv::Mat &image)
  {
    // TODO(第三讲)：实现装甲板检测，填写角点、大小板、编号与置信度。
    if (image.empty())
    {
      std::cout << "读取图片失败！" << std::endl;
      return {};
    }
    std::vector<Armor> Armors;
    cv::Mat draw_contours = image.clone();
    const cv::Mat blue_mask = createBlueMask(image);
    const std::vector<LightBar> light_bars = findLightBars(blue_mask, draw_contours);
    const std::vector<CandidatePair> candidates = findCandidatePairs(light_bars);
    Armors = drawMatchedPairs(draw_contours, light_bars, candidates, Armors);

    cv::imshow("contours result", draw_contours);
    cv::waitKey(1);
    return Armors;
  }
} // namespace training
