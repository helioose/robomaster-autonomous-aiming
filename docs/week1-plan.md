# 第 1 周学习计划（第二讲：仿真 IO）

> 适用：OpenCV 零基础、正在学 ROS2。
> 每天 1.5–2 小时，共约 12 小时。做不完就停在当天，不要往后赶。
> 仓库：`/home/huo/work/hnurmvision_edu`　分支：`feature/huo0136-intro`
>
> **看不懂命令先读：[day1-ros2-primer.md](day1-ros2-primer.md)**（三条环境命令、ROS2 核心概念、第 1 天逐步操作）

## 总体路线

```
ROS2 基础 ──┐
            ├─→ 第二讲 IO（本周）
cv::Mat 入门 ┘
      │
      └─→ OpenCV 图像处理 ─→ 第三讲 检测
                                  │
相机模型/PnP + Eigen ─────────────┴─→ 第四讲 位姿跟踪
                                          │
抛体运动/弹道 ────────────────────────────┴─→ 第五讲 瞄准
```

第二讲目标：把 `lessons/lesson02_io.cpp` 的四个空函数（`open/read/write/close`）实现，
让培训程序通过 ROS2 接收仿真图像与姿态、下发控制命令。

---

## 每天开始前：终端 C 的环境

另开一个终端专门跑 ROS2 命令，每次新开都执行：

```bash
cd /home/huo/work/hnurmvision_edu
source /opt/ros/humble/setup.bash
source simulation/install_interfaces/setup.bash
export ROS_DOMAIN_ID=89
```

运行程序仍用：

```bash
# 终端 A
./scripts/run_simulator.sh
# 终端 B
./scripts/run_training.sh
```

退出顺序：先关培训程序，再关仿真器。

---

## 第 1 天（10/26 周一）· 先把数据流“看见”

> 详细讲解见 [day1-ros2-primer.md](day1-ros2-primer.md)。

目标：不写代码，搞清楚仿真器到底在发什么。

- [ ] 终端 A 启动仿真器，终端 B 启动培训程序
- [ ] 确认空骨架行为：没有图像窗口、Ctrl+C 能退（这是正常的）
- [ ] 查看话题列表与消息类型：
  ```bash
  ros2 topic list
  ros2 topic info -v /image_raw
  ros2 topic hz /image_raw
  ros2 topic echo /vision_receive_data --once
  ```
- [ ] 用现成工具看画面（不写代码）：
  ```bash
  ros2 run image_tools showimage --ros-args -r image:=/image_raw
  ```
- [ ] 学：ROS2 的 node / topic / publisher / subscriber / QoS（Humble 官方 Beginner: CLI tools）

做完标志：能说清楚每个话题的「名称 / 消息类型 / 方向 / QoS / 频率」，手写一页笔记。

参考数据流：

| 话题 | 类型 | 方向 | QoS |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image`（rgb8，1440×1080） | 仿真器 → 我 | 默认 |
| `/vision_receive_data` | `hnurm_interfaces/VisionRecvData` | 仿真器 → 我 | SensorData |
| `/vision_send_data` | `hnurm_interfaces/VisionSendData` | 我 → 仿真器 | SensorData |

---

## 第 2 天（10/27 周二）· 消息结构 + cv::Mat 最小知识

目标：知道要填的四个函数该拿什么、还什么。

- [ ] 查看消息定义：
  ```bash
  ros2 interface show sensor_msgs/msg/Image
  ros2 interface show hnurm_interfaces/msg/VisionRecvData
  ros2 interface show hnurm_interfaces/msg/VisionSendData
  ```
- [ ] 阅读三个文件：
  - `io/io_context.hpp` —— 现在是个空结构体，需要自己加成员
  - `include/types.hpp` —— `SensorData` / `Command` 的字段含义
  - `src/main.cpp` —— 主循环怎么调用四个函数
- [ ] OpenCV 只学：`cv::Mat` 的 `rows/cols/type()/channels()/data`、`clone()`、
      `cvtColor`（`COLOR_RGB2BGR`）、`imshow` / `waitKey`
- [ ] 纸面设计 `IoContext` 成员：node、sub、pub、互斥锁、最新一帧缓存
- [ ] 写出 `open/read/write/close` 的伪代码

做完标志：能写出成员列表和四函数伪代码，不要求能编译。

---

## 第 3 天（10/28 周三）· 打通图像（里程碑 1）

目标：培训窗口显示仿真画面。

- [ ] 给 `io/io_context.hpp` 加成员
- [ ] `open()`：`rclcpp::init`、建节点、订阅 `/image_raw`（默认 QoS）
- [ ] `read()`：`spin_some` 取最新帧 → `rgb8` 转 BGR → `clone()` → 返回 `SensorData`
      （时间戳先随便填）
- [ ] 编译并运行：
  ```bash
  ./scripts/build_training.sh
  ```
- [ ] 排查常见坑：编码是 `rgb8` 不是 `bgr8`；忘了 `clone()` 图像会被覆写；
      订阅 QoS 不匹配；`rclcpp::init` 位置不对

做完标志：培训窗口出现仿真画面，并且仍能正常退出。
**这一步过了，第二讲最难的关就过了。**

---

## 第 4 天（10/29 周四）· 接入姿态和弹速 + 时间戳

目标：`SensorData` 四个字段全部有真实值。

- [ ] 订阅 `/vision_receive_data`，使用 `SensorDataQoS()`
- [ ] 取 `yaw/pitch/roll`（单位是**度**，转弧度）、`bullet_speed.data`（m/s）
- [ ] `timestamp` 改为图像 `header.stamp` 换算成 `TimePoint`，此后全项目统一用这个时钟
- [ ] 由 roll/pitch/yaw 构造 `imu_q`，先打印出来核对
- [ ] 在仿真里转动视角或云台，看数值是否跟着变

做完标志：姿态角、弹速合理且随仿真变化；弹速与 `simulation/simulator/config.toml`
里的 25 m/s 对得上。

---

## 第 5 天（10/30 周五）· 打通下发 + 安全门控

目标：能把命令送到仿真器，且 `allow_fire: false` 时绝不开火。

- [ ] 创建 `/vision_send_data` 发布者，使用 `SensorDataQoS()`
- [ ] `write()` 映射：
      `control` → `CONVERGING(1)`；`shoot` → `FIRE(2)`；
      无控制 → `LOST_TARGET(0)` 且 **`target_distance = -1`**
- [ ] `Command.yaw/pitch` 是弧度，消息里转成度
- [ ] 关键门控：`config.allow_fire == false` 时，无论上层 `shoot` 是什么都不能发 `FIRE`
- [ ] 验证：
  ```bash
  ros2 topic echo /vision_send_data --once
  ```
  当前主循环没有目标，应该看到 `LOST_TARGET`

做完标志：echo 能收到你发的消息；把 `allow_fire` 改成 true 再改回 false，确认判断生效。

---

## 第 6 天（10/31 周六）· 闭环测试云台

目标：你的命令能真正驱动仿真云台。

- [ ] **临时**造一个固定 `Command`（如 `control=true, yaw=0°, pitch=0°, shoot=false`）
- [ ] 观察仿真器云台是否转动，同时 `/vision_receive_data` 的 yaw/pitch 是否变化
- [ ] 理解：弧度/度在消息边界上的转换；“无目标”与 `target_distance = -1` 的语义
      （仿真器收到 -1 会忽略命令）
- [ ] 测完立刻删除临时代码

做完标志：云台按命令动起来，恢复原逻辑后一切正常。

---

## 第 7 天（11/1 周日）· 收尾、验收、提交

- [ ] 实现 `close()`：停 spin、释放订阅/发布者/节点、`rclcpp::shutdown()`
- [ ] 逐项验收（见下表）
- [ ] 提交到 `feature/huo0136-intro`，commit 说明完成第二讲 IO
- [ ] 写 20 分钟复盘：这周卡最久的点是什么（第三讲会用到）

验收表：

| 验收项 | 通过标准 | 完成 |
|---|---|---|
| 图像 | 培训窗口稳定显示仿真画面 | [ ] |
| 时间戳 | `timestamp` 来自图像 header，IMU/弹速同帧使用 | [ ] |
| 安全 | `allow_fire: false` 时绝不发 `FIRE` | [ ] |
| 退出 | 窗口 q/Esc、终端 Ctrl+C 都能干净退出，无卡死 | [ ] |
| 顺序 | 先退培训程序，再关仿真器 | [ ] |

---

## 关键文件速查

| 文件 | 作用 |
|---|---|
| `lessons/lesson02_io.cpp` | 本周要实现的四个函数 |
| `io/io_context.hpp` | `IoContext` 声明，需要加成员 |
| `include/types.hpp` | `SensorData` / `Command` 等数据类型 |
| `src/main.cpp` | 主循环，决定调用顺序 |
| `configs/simulation.yaml` | `display` / `allow_fire` / 相机参数 |
| `simulation/ros2/hnurm_interfaces/msg/` | 消息定义 |
| `simulation/simulator/src/ros2/plugin.rs` | 仿真器如何收发命令（对照理解） |

## 卡住时

- 超过 40 分钟没进展：先打印中间量，或回退到上一个能跑的版本
- 图像出不来：优先排查 QoS、`rgb8`/`bgr8`、`clone()`
- 每天记录一句：今天卡在哪、怎么解决的
