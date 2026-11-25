#pragma once
#include "config.hpp"
#include "types.hpp"
#include <optional>
#include <cstdint>
#include <deque>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<hnurm_interfaces/msg/vision_recv_data.hpp>
#include<hnurm_interfaces/msg/vision_send_data.hpp>
#include<hnurm_interfaces/msg/lesson_truth.hpp>

namespace training::io 
{
struct IoContext 
{
  Config config;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr feedback_sub;
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr  command_pub;
  sensor_msgs::msg::Image::SharedPtr image_latest;
  hnurm_interfaces::msg::VisionRecvData::SharedPtr feedback_latest;
  // 保存最近收到的姿态反馈，用图像时间戳寻找最接近的一帧姿态。
  std::deque<hnurm_interfaces::msg::VisionRecvData::SharedPtr> feedback_history;
  std::optional<std::int64_t> last_img_ns;

  // 第四讲验收演示：仿真器发布的装甲板真值，按图像时间戳配对。
  rclcpp::Subscription<hnurm_interfaces::msg::LessonTruth>::SharedPtr truth_sub;
  std::deque<hnurm_interfaces::msg::LessonTruth::SharedPtr> truth_history;

  void open();
  std::optional<SensorData> read();
  void write(const Command& command);
  void close();
};
}  // namespace training::io
