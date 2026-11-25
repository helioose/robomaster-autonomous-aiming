#include "io_context.hpp"
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>

namespace training::io
{
  namespace
  {
    // 35 Hz 下 128 条约可覆盖 3.6 秒的反馈历史，足以容纳图像与反馈的短时错位。
    constexpr std::size_t kFeedbackHistoryLimit = 128;
    // 真值按渲染帧发布，240 条足够覆盖图像与真值的短时错位。
    constexpr std::size_t kTruthHistoryLimit = 240;

    std::int64_t stampToNs(const builtin_interfaces::msg::Time &stamp)
    {
      return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
             static_cast<std::int64_t>(stamp.nanosec);
    }
  }

  void IoContext::open()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    last_img_ns.reset();
    feedback_history.clear();
    feedback_latest.reset();
    image_latest.reset();
    node = std::make_shared<rclcpp::Node>("training_io");
    image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::SharedPtr msg)
        {
          image_latest = msg;
        });
    feedback_sub = node->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
        "/vision_receive_data",
        rclcpp::QoS(10).best_effort(),
        [this](hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
        {
          feedback_latest = msg;
          feedback_history.push_back(msg);
          while (feedback_history.size() > kFeedbackHistoryLimit)
          {
            feedback_history.pop_front();
          }
        });
    truth_history.clear();
    truth_sub = node->create_subscription<hnurm_interfaces::msg::LessonTruth>(
        "/lesson/armor_truth",
        rclcpp::SensorDataQoS(),
        [this](hnurm_interfaces::msg::LessonTruth::SharedPtr msg)
        {
          truth_history.push_back(msg);
          while (truth_history.size() > kTruthHistoryLimit)
          {
            truth_history.pop_front();
          }
        });
    command_pub = node->create_publisher<hnurm_interfaces::msg::VisionSendData>(
        "/vision_send_data",
        10);
    std::cout << "通信管道全部接通！" << std::endl;
  }

  std::optional<SensorData> IoContext::read()
  {
    rclcpp::spin_some(node);

    // 没有图像，或还没有收到任何姿态反馈时，暂时无法组成完整输入。
    if (!image_latest || feedback_history.empty())
    {
      return std::nullopt;
    }

    const auto &stamp = image_latest->header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000u)
    {
      return std::nullopt;
    }

    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(stamp.nanosec);

    // 同一张图像只交给算法一次。
    if (last_img_ns && *last_img_ns == stamp_ns)
    {
      return std::nullopt;
    }

    // 在姿态历史中寻找采样时间与图像拍摄时间最接近的反馈。
    hnurm_interfaces::msg::VisionRecvData::SharedPtr matched_feedback;
    std::int64_t matched_feedback_stamp_ns = 0;
    std::int64_t min_abs_offset_ns = std::numeric_limits<std::int64_t>::max();

    for (const auto &feedback : feedback_history)
    {
      if (!feedback)
      {
        continue;
      }

      const auto &feedback_stamp = feedback->header.stamp;
      if (feedback_stamp.sec < 0 ||
          feedback_stamp.nanosec >= 1'000'000'000u)
      {
        continue;
      }

      const std::int64_t feedback_stamp_ns =
          static_cast<std::int64_t>(feedback_stamp.sec) * 1'000'000'000LL +
          static_cast<std::int64_t>(feedback_stamp.nanosec);

      const std::int64_t offset_ns = feedback_stamp_ns - stamp_ns;
      const std::int64_t abs_offset_ns =
          offset_ns >= 0 ? offset_ns : -offset_ns;

      if (abs_offset_ns < min_abs_offset_ns)
      {
        min_abs_offset_ns = abs_offset_ns;
        matched_feedback_stamp_ns = feedback_stamp_ns;
        matched_feedback = feedback;
      }
    }

    // 历史队列里没有合法的姿态时间戳。
    if (!matched_feedback)
    {
      return std::nullopt;
    }

    SensorData data;

    // 将 ROS SystemTime 时间戳映射到本程序使用的 steady_clock。
    // 仿真器的 ROS Clock 使用 SystemTime，因此两者共享 Unix 时间基准。
    const auto steady_before = Clock::now();
    const auto system_now = std::chrono::system_clock::now();
    const auto steady_after = Clock::now();

    // 用 steady_clock 两次采样的中点，减小取样先后造成的误差。
    const auto steady_now =
        steady_before + (steady_after - steady_before) / 2;

    const auto system_now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            system_now.time_since_epoch());

    const auto image_offset_from_now =
        std::chrono::nanoseconds{stamp_ns} - system_now_ns;

    data.timestamp =
        steady_now +
        std::chrono::duration_cast<Clock::duration>(image_offset_from_now);
    data.ros_stamp_ns = stamp_ns;

    // 真值与图像使用同一个 ROS 系统时钟，按 header.stamp 寻找最近的同一帧。
    const hnurm_interfaces::msg::LessonTruth::SharedPtr *matched_truth = nullptr;
    std::int64_t min_truth_offset_ns = std::numeric_limits<std::int64_t>::max();
    for (const auto &truth : truth_history)
    {
      if (!truth)
        continue;
      const auto &truth_stamp = truth->header.stamp;
      if (truth_stamp.sec < 0 || truth_stamp.nanosec >= 1'000'000'000u)
        continue;
      const std::int64_t offset_ns = stampToNs(truth_stamp) - stamp_ns;
      const std::int64_t abs_offset_ns = offset_ns >= 0 ? offset_ns : -offset_ns;
      if (abs_offset_ns < min_truth_offset_ns)
      {
        min_truth_offset_ns = abs_offset_ns;
        matched_truth = &truth;
      }
    }
    // 超过 200 ms 说明仿真器没有发真值（例如没开训练场景），不强行配对。
    if (matched_truth && min_truth_offset_ns <= 200'000'000LL)
    {
      for (const auto &armor : (*matched_truth)->armors)
      {
        TruthArmor truth_armor;
        truth_armor.id = armor.id;
        truth_armor.large_armor = armor.large_armor;
        for (std::size_t i = 0; i < 4; ++i)
        {
          truth_armor.corners[i] = cv::Point2f(
              static_cast<float>(armor.corners_px[i].x),
              static_cast<float>(armor.corners_px[i].y));
          truth_armor.corners_cam[i] =
              Eigen::Vector3d(armor.corners_m[i].x, armor.corners_m[i].y,
                              armor.corners_m[i].z);
        }
        truth_armor.position_cam = Eigen::Vector3d(
            armor.position.x, armor.position.y, armor.position.z);
        data.truth.push_back(truth_armor);
      }
    }

    // 使用与图像时间戳最近的反馈，而不是无条件使用最新收到的反馈。
    data.bullet_speed = matched_feedback->bullet_speed.data;

    const double roll_rad = matched_feedback->roll * (kPi / 180.0);
    const double pitch_rad = matched_feedback->pitch * (kPi / 180.0);
    const double yaw_rad = matched_feedback->yaw * (kPi / 180.0);

    // 保留原有姿态转换约定：不改变轴、角度单位或四元数乘法顺序。
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());
    data.imu_q = yawAngle * pitchAngle * rollAngle;

    // ROS Image 的 step 是每一行实际占用的字节数，不能假设它一定等于 width * 3。
    // 本项目仿真器发布 rgb8；同时兼容 bgr8，其他编码暂不处理。
    if (image_latest->encoding != "rgb8" && image_latest->encoding != "bgr8")
    {
      return std::nullopt;
    }

    const std::size_t width = image_latest->width;
    const std::size_t height = image_latest->height;
    const std::size_t step = image_latest->step;
    const std::size_t max_size = std::numeric_limits<std::size_t>::max();

    // OpenCV 的行列参数是 int，因此先检查尺寸范围和乘法溢出。
    if (width == 0 || height == 0 ||
        width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        width > max_size / 3)
    {
      return std::nullopt;
    }

    const std::size_t min_step = width * 3;
    if (step < min_step || step == 0 || height > max_size / step)
    {
      return std::nullopt;
    }

    const std::size_t required_bytes = step * height;
    if (image_latest->data.size() < required_bytes)
    {
      return std::nullopt;
    }

    // 用 ROS 消息提供的 step 构造视图，正确处理可能存在的行尾填充字节。
    // source 只引用 ROS 消息内存；下面 clone/cvtColor 后，data.image 独立拥有像素内存。
    const cv::Mat source(
        static_cast<int>(height),
        static_cast<int>(width),
        CV_8UC3,
        image_latest->data.data(),
        step);

    if (image_latest->encoding == "rgb8")
    {
      // ROS 仿真器发布 RGB，转换成项目 SensorData 约定的 OpenCV BGR。
      cv::cvtColor(source, data.image, cv::COLOR_RGB2BGR);
    }
    else
    {
      // 已经是 BGR 时无需换通道，但仍需复制，确保 data.image 独立拥有内存。
      data.image = source.clone();
    }

    const double feedback_offset_ms =
        (matched_feedback_stamp_ns - stamp_ns) / 1'000'000.0;

    // 临时诊断输出：显示实际匹配到的姿态时间差和图像年龄。
    static int count_frame = 0;
    ++count_frame;
    if (count_frame >= 25)
    {
      count_frame = 0;
      std::cout << "\nyaw=" << matched_feedback->yaw << " deg\n"
                << "pitch=" << matched_feedback->pitch << " deg\n"
                << "bullet_speed="
                << static_cast<unsigned int>(matched_feedback->bullet_speed.data)
                << " m/s\n"
                << "matched_feedback_offset=" << feedback_offset_ms << " ms\n";

      const double image_age_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - data.timestamp)
              .count();
      std::cout << "image_age=" << image_age_ms << " ms\n";
    }

    last_img_ns = stamp_ns;
    return data;
  }

  void IoContext::write(const Command &command)
  {

    hnurm_interfaces::msg::VisionSendData msg;
    msg.target_state.data = 0;
    msg.target_distance = 0.0;
    if (std::isfinite(command.yaw) && std::isfinite(command.pitch))
    {
      if (command.control)
      {
        msg.target_distance = -1;
        if (command.shoot && config.allow_fire)
          msg.target_state.data = 2;
        else
          msg.target_state.data = 1;
        msg.pitch = command.pitch * (180.0 / kPi);
        msg.yaw = command.yaw * (180.0 / kPi);
      }
    }
    msg.header.stamp = node->now();
    msg.header.frame_id = "gimbal_link";

    if (command_pub)
    {
      command_pub->publish(msg);
    }
  }

  void IoContext::close()
  {
    Command stop_command{};
    write(stop_command);
    if (node)
      node.reset();
    rclcpp::shutdown();
    std::cout << "IO Close: 安全退出，云台已停止。" << std::endl;
  }
} // namespace training::io