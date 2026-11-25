# 第四讲验收：仿真器发布的“同一帧真实可见装甲板”真值。
# header.stamp 与同一时间的 /image_raw 完全相同，可直接按时间戳配对。

std_msgs/Header header
geometry_msgs/Pose camera        # 相机在 map 系的位姿（map 即 IMU 稳定系）
LessonArmorObservation[] armors
