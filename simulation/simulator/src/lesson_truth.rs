//! 第四讲验收：发布与 `/image_raw` 同帧的装甲板真值。
//!
//! 真值直接来自渲染相机实际可见的装甲板灯条角点（`MarkerData`）：
//! - 每个角点变到相机光学坐标系（x 右、y 下、z 前），单位 m；
//! - 同时按相机内参投影像素，学员可以把检测框和真值框画在同一张图上；
//! - 中心点也给出同样两种表示：光学系（m）和像素。
//!
//! 话题：`/lesson/armor_truth`，`hnurm_interfaces/msg/LessonTruth`。
//! 时间戳与 `/image_raw` 使用同一个 ROS 系统时钟，可按 header.stamp 直接配对。
//!
//! 注意：本消息只给“相机坐标系”真值，不给 IMU 系真值。学员用自己那套
//! `camera_to_gimbal` / `imu_q` 把 PnP 结果反变换回相机系后，和这里的真值逐轴
//! 比较，就能看出 PnP 本身以及坐标换算链是否接反。

use crate::capture::CaptureSource;
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{Armor, ArmorParts, ArmorType, MarkerData, Team};
use crate::ros2::prelude::transform;
use crate::ros2::topic::{LessonTruthTopic, TopicPublisher};
use bevy::prelude::*;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion};
use r2r::hnurm_interfaces::msg::{LessonArmorObservation, LessonTruth};
use r2r::std_msgs::msg::Header;

pub struct LessonTruthPlugin;

impl Plugin for LessonTruthPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Last, publish_lesson_truth);
    }
}

/// 与 `/image_raw` 相同的时钟源（ROS 系统时间）。
fn now_stamp() -> Time {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    Time {
        sec: now.as_secs() as i32,
        nanosec: now.subsec_nanos(),
    }
}

/// Bevy 相机局部坐标（-Z 前、Y 上、X 右）→ ROS 光学坐标（+Z 前、Y 下、X 右）。
fn to_optical(cam_local: Vec3) -> [f64; 3] {
    [
        cam_local.x as f64,
        -cam_local.y as f64,
        -cam_local.z as f64,
    ]
}

/// 相机内参（仿真器无畸变；与 capture.rs 的 compute_camera_intrinsics 同式）。
struct Intrinsics {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    width: u32,
    height: u32,
}

impl Intrinsics {
    fn new(fov_deg: f32, width: u32, height: u32) -> Self {
        let fov_y = fov_deg as f64 / 180.0 * std::f64::consts::PI;
        let aspect = width as f64 / height as f64;
        let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();
        Self {
            fx: width as f64 / (2.0 * (fov_x / 2.0).tan()),
            fy: height as f64 / (2.0 * (fov_y / 2.0).tan()),
            cx: width as f64 / 2.0,
            cy: height as f64 / 2.0,
            width,
            height,
        }
    }

    fn project(&self, p: &[f64; 3]) -> Point {
        Point {
            x: self.fx * p[0] / p[2] + self.cx,
            y: self.fy * p[1] / p[2] + self.cy,
            z: 0.0,
        }
    }

    fn inside_with_margin(&self, p: &Point, margin: f64) -> bool {
        p.x > -margin
            && p.x < self.width as f64 + margin
            && p.y > -margin
            && p.y < self.height as f64 + margin
    }
}

pub fn publish_lesson_truth(
    config: Res<SimulationConfig>,
    camera: Single<&GlobalTransform, With<CaptureSource>>,
    armors: Query<(&Armor, &ArmorParts)>,
    markers: Query<(&GlobalTransform, &MarkerData)>,
    mut publisher: ResMut<TopicPublisher<LessonTruthTopic>>,
) {
    let cam_inv = camera.affine().inverse();
    let cam_ros = transform(camera.compute_transform());

    if !config.training.enabled {
        return;
    }

    let k = Intrinsics::new(
        config.camera.fov,
        config.capture.color.width,
        config.capture.color.height,
    );

    let mut observations = Vec::new();
    for (armor, parts) in &armors {
        // 观察车是红方，训练靶是蓝方；只发布蓝方装甲板。
        if armor.team != Team::Blue {
            continue;
        }
        let Ok((marker_tf, MarkerData(points))) = markers.get(parts.marker()) else {
            continue;
        };

        let mut corner_optical = Vec::with_capacity(4);
        let mut corner_px = Vec::with_capacity(4);
        let mut center = [0.0_f64; 3];
        let mut visible = true;
        for point in points {
            let world = marker_tf.transform_point(*point);
            let optical = to_optical(cam_inv.transform_point3(world));
            if optical[2] <= 0.1 {
                visible = false;
                break;
            }
            for axis in 0..3 {
                center[axis] += optical[axis] / 4.0;
            }
            corner_px.push(k.project(&optical));
            corner_optical.push(optical);
        }
        if !visible {
            continue;
        }

        let center_px = k.project(&center);
        // 停放/视野外的目标不发布。
        if !k.inside_with_margin(&center_px, 100.0) {
            continue;
        }

        // r2r 把定长数组也映射成 Vec，这里保持顺序即可（与 marker 角点顺序一致）。
        let corners_px: Vec<Point> = corner_px;
        let corners_m: Vec<Point> = corner_optical
            .iter()
            .map(|p| Point { x: p[0], y: p[1], z: p[2] })
            .collect();

        observations.push(LessonArmorObservation {
            id: armor_id(armor, parts),
            large_armor: matches!(armor.spec.armor_type(), ArmorType::Large),
            label: armor.label as i32,
            corners_px,
            corners_m,
            position: Point {
                x: center[0],
                y: center[1],
                z: center[2],
            },
            position_px: center_px,
            range_m: center[2],
        });
    }

    let truth = LessonTruth {
        header: Header {
            stamp: now_stamp(),
            frame_id: "camera_optical_frame".to_string(),
        },
        camera: Pose {
            position: Point {
                x: cam_ros.translation.x,
                y: cam_ros.translation.y,
                z: cam_ros.translation.z,
            },
            orientation: Quaternion {
                x: cam_ros.rotation.x,
                y: cam_ros.rotation.y,
                z: cam_ros.rotation.z,
                w: cam_ros.rotation.w,
            },
        },
        armors: observations,
    };
    publisher.publish(truth);
}

/// 同一标签、同一大小的多个装甲模块视为同一目标的四个面，因此 id 只由队伍与标签决定。
fn armor_id(armor: &Armor, _parts: &ArmorParts) -> u32 {
    let team = match armor.team {
        Team::Red => 0,
        Team::Blue => 1,
    };
    ((team as u32) << 8) | (armor.label as u32 & 0xff)
}
