#pragma once
#include "config.hpp"
#include "types.hpp"
#include <optional>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<hnurm_interfaces/msg/vision_recv_data.hpp>
#include<hnurm_interfaces/msg/vision_send_data.hpp>

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

  void open();
  std::optional<SensorData> read();
  void write(const Command& command);
  void close();
};
}  // namespace training::io
