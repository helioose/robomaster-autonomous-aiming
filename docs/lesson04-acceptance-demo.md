# 第四讲验收演示：PnP 位姿对比真值 + 平移靶预测

演示程序把“学员的第三讲检测、第四讲跟踪”和“仿真器发布的真值”画在同一帧上，
按验收要求录成 MP4。

## 验收要求与对应画面

| 要求 | 画面表现 |
| --- | --- |
| 静止靶：PnP 解算位姿与真值比对、计算误差 | 黄框（PnP）、绿框（真值）；左下角 `STATIC PnP vs TRUTH \|d\|=…` 与逐轴差值、重投影误差 |
| 平移靶：预测 n 毫秒后的位置，用另一种颜色的框画出 | 品红框 = `t+n` 预测位置（n 来自配置 `prediction_ms`，默认 250） |
| 250 ms 后真值与预测比对，误差显示在左下角 | 真值到达 `t+n` 的那一帧换成橙框，左下角 `TRANSLATE PREDICT(t+250ms) vs TRUTH \|d\|=…`，并给出均值 / 最大值 |

## 坐标系与时间

- 真值话题 `/lesson/armor_truth`（`hnurm_interfaces/msg/LessonTruth`）与 `/image_raw`
  使用同一个 ROS 系统时钟，演示程序按 `header.stamp` 把真值与图像配对（容忍 200 ms）。
- 真值只给“相机光学系”（x 右、y 下、z 前，单位 m）。比较 PnP 与真值时都在相机系里做，
  不受 IMU 姿态换算链影响；预测在 IMU 稳定系里按跟踪器外推，结算时把真值经
  `R_camera2gimbal`、`imu_q` 旋到 IMU 系再比较（与 `solveArmorPose` 的链路严格互逆）。
- 预测时刻 `t+n` 的判定：真值时间戳首次 ≥ `t+n` 的那一帧。
- 演示只为“锁定的一块板”做预测：锁定目标是画面里最近的蓝方装甲板（导论的“哪个近打哪个”），
  跟踪器也只收到这一块板的观测，避免开局因远处小板重投影误差更小而锁错车。
- 结算时若真值比目标时刻晚 150 ms 以上（掉帧），或最近同 id 真值离预测超过 1 m
  （画面里同标签的远处装甲板），则丢弃该次预测，不计入均值。

## 运行

```bash
# 终端 1：仿真器
./scripts/run_simulator.sh
#   场景内按 F7 开始计时；静止靶是默认的第 0 号靶，平移靶按 F6 切换

# 终端 2：演示程序（先静止靶，再平移靶）
source /opt/ros/humble/setup.bash
source simulation/install_interfaces/setup.bash
./build/training configs/lesson04_static.yaml
./build/training configs/lesson04_prediction.yaml
```

`q` 或 Esc 结束并写出 MP4。

## 关键实现位置

| 内容 | 位置 |
| --- | --- |
| 仿真器发布真值 | `simulation/simulator/src/lesson_truth.rs` |
| 真值订阅与按帧配对 | `io/io_context.hpp`、`lessons/lesson02_io.cpp` |
| 演示主循环（画框、HUD、录像、结算） | `src/main.cpp` 的 `runDemo` |
| 画框 / HUD 小工具 | `include/lesson04_demo.hpp`、`lessons/lesson04_demo.cpp` |
| 配置字段 | `io/config.hpp`、`io/config.cpp`（`demo`、`demo_pnp_source`、`prediction_ms`、`record`、`record_path`） |

演示模式不进入正常五讲流水线：`demo: true` 时 `main` 走 `runDemo`。
