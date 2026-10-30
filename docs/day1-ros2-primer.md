# 第 1 天详解：ROS2 入门与仿真数据流

> 配合 [week1-plan.md](week1-plan.md) 使用。看不懂计划里的命令时，先读这一篇。
> 本篇只讲第 1 天需要的概念，不涉及写代码。

---

## 一、“每天开始前”那三条命令到底在干什么

新开的终端就像一台刚重启的电脑：它不知道 ROS2 装在哪，也不知道本仓库的自定义消息在哪。
下面三条命令就是给这个终端“装好环境”，**每开一个新终端都要执行一次**。

```bash
cd /home/huo/work/hnurmvision_edu
source /opt/ros/humble/setup.bash
source simulation/install_interfaces/setup.bash
export ROS_DOMAIN_ID=89
```

### 1. `source /opt/ros/humble/setup.bash`

`source` 的作用是**执行一个脚本，并让它设置的变量留在当前终端里**。

这个脚本会设置 `PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH` 等环境变量，让终端能找到：

- `ros2` 命令本身；
- ROS2 自带的 `sensor_msgs` 等消息；
- ROS2 的 C++/Python 运行库。

不执行它，输入 `ros2` 会提示 `command not found`。可以把这一步理解成“打开 ROS2 开关”。

### 2. `source simulation/install_interfaces/setup.bash`

ROS2 自带的 `sensor_msgs` 由第 1 条搞定，但本项目还有**自定义消息** `hnurm_interfaces`
（`VisionRecvData`、`VisionSendData` 等）。它们编译后安装在：

```
simulation/install_interfaces/
```

必须再 source 一次，终端才知道这些消息的存在。否则：

- `ros2 interface show hnurm_interfaces/msg/VisionRecvData` 会报找不到包；
- 培训程序运行时也找不到消息的类型支持，无法和仿真器通信。

注意路径是相对的，所以要先 `cd` 到仓库根目录。

### 3. `export ROS_DOMAIN_ID=89`

ROS2 底层用 DDS 做通信。DDS 把网络划分成多个“域（domain）”，**只有域号相同的节点才能互相发现**。

- 仿真器脚本 `run_simulator.sh` 和培训脚本 `run_training.sh` 都设置了 `ROS_DOMAIN_ID=89`；
- 新开的终端默认是 `0`。

所以如果终端 C 不设这个变量，就会出现“仿真器明明在跑，`ros2 topic list` 却空空如也”的经典问题。

验证：

```bash
echo $ROS_DOMAIN_ID    # 应输出 89
```

### 4. 为什么要专门开一个“终端 C”

- 终端 A 被仿真器占着，终端 B 被培训程序占着，两个都不能再输入命令；
- 终端 C 专门用来“观察”和“查询”，可以反复执行 `ros2` 命令，不影响另外两个程序。

终端 C 不需要一直开着，但每次重新打开都要重复上面四条命令。

---

## 二、看懂 ROS2 的 5 个概念

用“电台广播”来理解：

| 概念 | 通俗解释 | 在本项目里 |
|---|---|---|
| Node（节点） | 一个参与通信的程序 | 仿真器是一个节点，你的培训程序是另一个节点 |
| Topic（话题） | 一个有名字的广播频道 | `/image_raw`、`/vision_send_data` |
| Message（消息） | 频道里播报的数据格式（有固定字段） | `sensor_msgs/Image`、`VisionRecvData` |
| Publisher（发布者） | 往频道里说话的一方 | 仿真器发布图像；你发布控制命令 |
| Subscriber（订阅者） | 收听频道的一方 | 你订阅图像；仿真器订阅你的命令 |

几个补充：

- **发布者和订阅者互不认识**：仿真器只管往 `/image_raw` 发，谁在听、有没有人听，它不关心。
  这解释了为什么培训程序还没写订阅时，仿真器照样在发图像。
- **QoS（服务质量）**：收发双方对“可靠性、缓存深度”的约定。
  图像用默认 QoS；`vision_receive_data` / `vision_send_data` 用 `sensor_data`（尽力而为）。
  约定不匹配时，双方能互相看见话题，但收不到数据。
- **`header.stamp`**：这条消息产生的时间，后面做时间同步要用它。

### 本项目的完整数据流

```
                  ┌─────────────────────────────┐
                  │        仿真器节点            │
                  └─────────────────────────────┘
   发布 /image_raw (Image)              ──────────►  你的订阅者
   发布 /vision_receive_data            ──────────►  你的订阅者
        (VisionRecvData：姿态/弹速)
   订阅 /vision_send_data               ◄──────────  你的发布者
        (VisionSendData：控制/发射)
```

**第二讲要做的，就是把右边这三个“你的订阅者/发布者”实现出来**，位置在
`lessons/lesson02_io.cpp` 的 `open()` 和 `write()` 里。

---

## 三、第 1 天逐步操作（照着做）

### 步骤 1：启动两个程序

```bash
# 终端 A
cd /home/huo/work/hnurmvision_edu
./scripts/run_simulator.sh

# 终端 B
cd /home/huo/work/hnurmvision_edu
./scripts/run_training.sh
```

预期结果：

- 终端 A：出现仿真窗口，能看到训练场地和靶车；
- 终端 B：培训程序在空转，**没有图像窗口**。这是正常的，因为
  `read()` 永远返回“没有数据”，`main.cpp` 就不会调用 `imshow`。

在终端 B 按 `Ctrl+C`，程序应当能退出。

### 步骤 2：确认终端 C 能看到话题

```bash
ros2 topic list
```

预期能看到（顺序不限）：

```
/camera_info
/camera_pose
/gimbal_pose
/image_compressed
/image_raw
/muzzle_pose
/odom_pose
/parameter_events
/rosout
/simulator/marker
/simulator/power_rune/hit
/simulator/power_rune/state
/simulator/tech_core/state
/tf
/vision_receive_data
/vision_send_data
```

如果列表是空的，按这个顺序排查：

1. 仿真器是否真的在跑？
2. 终端 C 是否 `export ROS_DOMAIN_ID=89`？
3. 是否 source 了两个 `setup.bash`？

### 步骤 3：看清一个话题的“收发双方”

```bash
ros2 topic info -v /image_raw
```

你会看到类似：

```
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
...
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  History: KEEP_LAST
  Depth: ...
```

重点看 **Subscription count: 0**：仿真器一直在发图，但**没有任何人在听**。
这就是培训窗口不显示图像的原因，也是第二讲存在的意义。

### 步骤 4：确认数据真的在流动

```bash
ros2 topic hz /image_raw
```

预期每行显示一个接近渲染帧率的值（如 50–60 Hz）。按 `Ctrl+C` 停止。
如果一直没有输出，回到步骤 2 的排查。

### 步骤 5：读一条姿态消息

```bash
ros2 topic echo /vision_receive_data --once
```

`--once` 表示收到一条就退出。预期输出类似：

```yaml
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: gimbal_link
self_color:
  data: 1
work_mode:
  data: 0
bullet_speed:
  data: 25
roll: ...
pitch: ...
yaw: ...
...
```

对照理解：

- `yaw / pitch / roll` 单位是**度**，来自云台姿态；
- `bullet_speed.data = 25` 是弹速（m/s），对应仿真配置里的 `speed = 25.0`；
- `frame_id: gimbal_link` 表示这些角度在云台坐标系下；
- 只取某个字段可以用 `--field`，例如：
  ```bash
  ros2 topic echo /vision_receive_data --field yaw --once
  ```

这些字段就是第 4 天要填进 `SensorData.imu_q` 和 `bullet_speed` 的原料。

### 步骤 6：不写代码先看一眼图像

```bash
ros2 run image_tools showimage --ros-args -r image:=/image_raw
```

- `ros2 run 包名 程序名`：运行某个 ROS2 包里的程序；
- `--ros-args -r image:=/image_raw`：把 `showimage` 内部默认订阅的 `image` 话题，
  重映射（remap）到仿真器的 `/image_raw`。

预期弹出一个窗口显示仿真相机画面。看完关闭窗口，或按 `Ctrl+C`。

如果报错打不开窗口，检查是不是在桌面图形终端里执行（需要 `DISPLAY` 环境变量，
本机是 `:1`），不要用 SSH 或纯文字终端。

### 步骤 7（可选）：看看“我还没写的发布者”

```bash
ros2 topic info -v /vision_send_data
```

预期 **Publisher count: 0、Subscription count: 1**：
仿真器在等你的命令，但还没有人发。等第 5 天你写完 `write()`，Publisher count 就会变成 1。

### 步骤 8：整理今天的笔记

用 10 分钟写下这张表（不要抄，自己回忆着填）：

| 话题 | 消息类型 | 谁发布 | 谁订阅 | 方向 |
|---|---|---|---|---|
| `/image_raw` | | | | |
| `/vision_receive_data` | | | | |
| `/vision_send_data` | | | | |

---

## 四、第 1 天要学的内容

### 必须学（约 30–40 分钟）

ROS2 Humble 官方 Tutorials 的 Beginner 部分，按顺序看这几节：

1. **Understanding nodes** —— 节点是什么，`ros2 node list`、`ros2 node info`
2. **Understanding topics** —— 话题、发布者、订阅者，`ros2 topic list/info/echo/hz`
3. **Understanding interfaces**（或 `ros2 interface` 相关章节）—— 消息定义怎么读
4. **Understanding QoS**（概念/Intermediate 章节）—— 只需知道 reliable / best_effort 的区别

配套命令练习：

```bash
ros2 node list
ros2 node info /simulator
ros2 topic list -t
ros2 interface show sensor_msgs/msg/Image
```

### 今天先不要学

- launch 文件、service、action、parameter、生命周期节点
- TF2 坐标变换（第三、四讲再学）
- C++ 订阅/发布代码写法（第 3 天再学）
- OpenCV（今天完全不需要）

### 自测 5 问（能答上就算过关）

1. 仿真器和培训程序分别是什么节点？
2. `/image_raw` 有发布者但培训窗口不显示，原因是什么？
3. 为什么 `ros2 topic list` 为空时，第一个要检查的是 `ROS_DOMAIN_ID`？
4. 为什么需要 source 两个 `setup.bash`？
5. `sensor_data` QoS 和默认 QoS 有什么区别？本项目哪些话题用哪一个？

### 时间安排建议

| 内容 | 时间 |
|---|---|
| 读本节 + 官方节点/话题教程 | 30 分钟 |
| 步骤 1–7 实际操作 | 40–50 分钟 |
| 填笔记 + 自测 5 问 | 15 分钟 |

### 补写 lesson02 前要读的文件

不需要读完整个仓库。lesson02 只实现 `IoContext` 的 `open/read/write/close`，读完下面这些就够动手了。

**代码契约（先读，约 30 分钟）**

| 文件 | 要拿到什么 |
|---|---|
| `README.md` | 构建、两个终端、运行与退出方式 |
| `include/types.hpp` | `SensorData`、`Command`、`CameraCalibration` 的字段和单位约定 |
| `include/lessons.hpp`、`src/main.cpp` | 调用顺序：`open` → 循环 `read` → 算法 → `write` → `close`；`read` 返回 `nullopt` 表示本帧无数据；显示由 `main` 负责 |
| `io/io_context.hpp`、`io/config.hpp`、`io/config.cpp` | IO 状态、配置加载与校验 |
| `lessons/lesson02_io.cpp` | 待补写的四个空函数 |
| `configs/simulation.yaml` | `display`、`allow_fire`、相机内参和坐标变换 |

**ROS2 接口定义（写 `read`/`write` 前必须查）**

- `simulation/ros2/hnurm_interfaces/msg/uart/VisionRecvData.msg` —— IMU 姿态等上行数据
- `simulation/ros2/hnurm_interfaces/msg/uart/VisionSendData.msg` —— 下发指令
- `simulation/ros2/hnurm_interfaces/msg/uart/BulletSpeed.msg` —— 弹速
- `simulation/ros2/hnurm_interfaces/srv/SetWorkMode.srv`、`srv/SetSelfColor.srv` —— `open()` 里的模式设置
- `simulation/ros2/hnurm_interfaces/msg/uart/WorkMode.msg`、`SelfColor.msg` —— 上面两个 service 用到的枚举
- 根目录 `CMakeLists.txt`、`scripts/run_training.sh` —— 链接了哪些 ROS2 库、运行时环境变量

**明确不用读**

- `lessons/lesson03_detection.cpp` ~ `lesson05_aiming.cpp`、`include/tracking_state.hpp` —— 下游算法，接口已由 `lessons.hpp` 固定
- `simulation/simulator/`（Rust 源码、资产）、`simulation/tools/`、`assets/`、`build/`

**易错点**：`types.hpp` 的 `TimePoint` 是 `std::chrono::steady_clock`，ROS2 消息时间戳一般是 system clock，`read()` 里要做一次转换；`Command` 的 yaw/pitch 单位与方向也在 `write()` 边界统一转换，不要留给 lesson05。

---

## 五、常见问题

**Q：`ros2: command not found`**
→ 没 source `/opt/ros/humble/setup.bash`，或新开了终端忘了重来一遍。

**Q：`ros2 topic list` 是空的**
→ 依次检查：仿真器在跑吗？`echo $ROS_DOMAIN_ID` 是 89 吗？两个 `setup.bash` 都 source 了吗？

**Q：`ros2 interface show hnurm_interfaces/...` 找不到包**
→ 没 source `simulation/install_interfaces/setup.bash`，或者不在仓库根目录。

**Q：`ros2 topic echo` 一直没输出**
→ 话题名写错、`ROS_DOMAIN_ID` 不一致，或 QoS 不匹配。可先 `ros2 topic info -v` 看收发双方是否都存在。

**Q：`ros2 topic echo /image_raw` 刷屏怎么办**
→ 不要对图像话题用 `echo`，数据量很大。用 `showimage`，或 `ros2 topic hz /image_raw`。

**Q：`showimage` 打不开窗口**
→ 必须在桌面图形会话里运行（本机 `DISPLAY=:1`）。纯文字终端或 SSH 不行。
