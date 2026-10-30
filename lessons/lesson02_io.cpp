#include "io_context.hpp"
#include <chrono>
#include <thread>
#include<cstdint>
#include<opencv2/opencv.hpp>
#include<Eigen/Geometry>

namespace training::io 
{
  void IoContext::open() 
  {
    if(!rclcpp::ok())
    {
      rclcpp::init(0,nullptr);
    }
    node=std::make_shared<rclcpp::Node>("training_io");
    image_sub=node->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg)
      {
        image_latest=msg;
      }
    );
    feedback_sub=node->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
      "/vision_receive_data",
      rclcpp::QoS(10).best_effort(), 
      [this](hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
      {
        feedback_latest=msg;
      }
    );
    command_pub=node->create_publisher<hnurm_interfaces::msg::VisionSendData>(
      "/vision_send_data",
      10
    );
    std::cout<<"通信管道全部接通！"<<std::endl;
  }


  std::optional<SensorData> IoContext::read() 
  {
    rclcpp::spin_some(node);
    if(!feedback_latest||!image_latest)
    {
      return std::nullopt;
    }

    SensorData data;

    data.bullet_speed =feedback_latest->bullet_speed.data;

    const auto& stamp=image_lastest->header.stamp;
    if(stamp.sec<0||stamp>=1'000'000'000u) return std::nullopt;
    const std::int64_t stamp_ns=
      static_cast<std::int64_t>(stamp.sec)*1'000'000'000LL
        +static_cast<std::int64_t>(stamp.nanosec);
    data.timestamp=TimePoint(
      std::chrono::duration_cast<Clock::duration>(
        std::chrono::nanoseconds{stamp_ns}
      )
    );
    
    double roll_rad = feedback_latest->roll * (kPi / 180.0);
    double pitch_rad = feedback_latest->pitch * (kPi / 180.0);
    double yaw_rad = feedback_latest->yaw * (kPi / 180.0);

    Eigen::AngleAxisd rollAngle(roll_rad,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad,Eigen::Vector3d::UnitZ());
    data.imu_q=yawAngle*pitchAngle*rollAngle;

    data.image=cv::Mat(
      image_latest->height,
      image_latest->width,
      CV_8UC3,
      image_latest->data.data()
    ).clone();

    return data;
  }

void IoContext::write(const Command& command) {

  hnurm_interfaces::msg::VisionSendData msg;
  msg.target_state.data=0;
  msg.target_distance=0.0;
  if(std::isfinite(command.yaw) && std::isfinite(command.pitch))
  {
    if(command.control)
    {
      msg.target_distance=-1;
      if(command.shoot && config.allow_fire) msg.target_state.data=2;
      else msg.target_state.data=1;
      msg.pitch=command.pitch*(180.0/kPi);
      msg.yaw=command.yaw*(180.0/kPi);
    }
  }
  msg.header.stamp=node->now();
  msg.header.frame_id="gimbal_link";

  if(command_pub) 
  { 
    command_pub->publish(msg);
  }
}

void IoContext::close() 
{
  Command stop_command{};
  write(stop_command);
  if(node) node.reset();
  rclcpp::shutdown();
  std::cout << "IO Close: 安全退出，云台已停止。" << std::endl;
}
}  // namespace training::io
