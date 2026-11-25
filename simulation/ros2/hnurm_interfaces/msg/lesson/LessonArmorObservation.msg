# 一块在图像中真实可见的装甲板（仿真侧真值），供第四讲 PnP 验收比对。
# 与 /image_raw 同一帧时间戳；坐标系为相机光学系（x 右、y 下、z 前，单位 m）。

uint32 id                # 队伍+标签，用来在帧间关联同一块装甲板
bool large_armor         # 大装甲板 / 小装甲板
int32 label              # ArmorLabel 枚举值（2/3/4 等）

# 灯条上的四个角点，与 Armor::corners 同序：左上、右上、右下、左下
geometry_msgs/Point[4] corners_px     # 像素
geometry_msgs/Point[4] corners_m      # 相机光学系，m
geometry_msgs/Point position          # 装甲板中心，相机光学系，m
geometry_msgs/Point position_px       # 装甲板中心，像素
float64 range_m                       # 沿光轴到装甲板中心的距离，m
