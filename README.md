# Big Bertha Bringup

ROS 2 **Jazzy** bringup for the **Big Bertha** quadruped: a PPO locomotion policy
(exported to ONNX, run from a C++ node) drives a learned gait, while SLAM builds a
map and Nav2 plans collision-free point-A-to-point-B paths — demonstrated in a
Gazebo Harmonic world full of obstacles.

> Simulation bringup first; hardware bringup is scaffolded but empty.
> Everything is built **for Big Bertha** for now (URDF, meshes, weights).

See **[PLAN.md](./PLAN.md)** for the full architecture, module decomposition,
diagrams, CI, and execution plan.

---

## Packages

| Package | Type | Role |
|---|---|---|---|
| [`spider_msgs`](./spider_msgs) | `ament_cmake` | Robot-agnostic interfaces (shared with the future Lil Spider) |
| [`big_bertha_description`](./big_bertha_description) | `ament_cmake` | URDF/xacro, meshes, `ros2_control` |
| [`big_bertha_policy_controller`](./big_bertha_policy_controller) | `ament_cmake` (C++) | ONNX gait node: `/cmd_vel` → 12 joint targets |
| [`big_bertha_sim_bringup`](./big_bertha_sim_bringup) | `ament_cmake` | Gazebo sim: world, SLAM, Nav2, RViz |
| [`big_bertha_hardware_bringup`](./big_bertha_hardware_bringup) | `ament_python` | Hardware bringup: MPU6050 + PCA9685 via Bridge |

## Quick start (simulation)

```bash
# 1. Install the stack (once)
bash scripts/install_jazzy.sh

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash

# 3. Bring up the full sim (obstacle world + gait + SLAM + Nav2 + RViz)
ros2 launch big_bertha_sim_bringup bringup.launch.py
```

## Autonomy stack (functional modules)

```
description → simulation → locomotion → state_estimation → mapping → localization → planning
```

## Hardware target

Arduino UNO Q (4 GB, ROS 2 Jazzy, **arm64**) · 3D-printed frame · 12× MG995 servos ·
1× YDLidar X2 · 1× MPU6050 IMU. (Hardware bringup is future work; the arm64 CI leg
exists because the deploy target is arm64.)

---
## Progress

| # | Module | Status |
|---|---|---|
| 0 | Environment, scaffold & CI | ✅ Done |
| 1 | `spider_msgs` interfaces | ✅ Done |
| 2 | URDF / description / meshes | ✅ Done |
| 3 | Simulation (Gazebo world + bridge) | ✅ Done |
| 4 | Locomotion (C++ ONNX gait controller) | ✅ Done |
| 5 | MPU6050 + PCA9685 driver | ✅ Done |
| 6 | Leg odometry | ❌ Not started |
| 7 | State estimation (robot_localization EKF) | ❌ Not started |
| 8 | Mapping (slam_toolbox) | ❌ Not started |
| 9 | Localization (AMCL + map_server) | ❌ Not started |
| 10 | Planning (Nav2) | ❌ Not started |
| 11 | Integration + end-to-end verification | ❌ Not started |

### Hardware bringup order

1. ✅ **MPU6050 + PCA9685 driver** — combined ROS 2 bridge node (`hardware_bridge`)
2. ❌ **Leg odometry** — compute `/odom` (`nav_msgs/Odometry`) from joint kinematics
3. ❌ **YDLidar X2** — `ydlidar_ros2_driver` → `/scan` (`sensor_msgs/LaserScan`)
4. ❌ **State estimation → mapping → localization → planning** (identical config to sim, `use_sim_time:=false`)

---

## Running the bridge node

```bash
ros2 launch big_bertha_hardware_bringup hardware_bridge.launch.py
```

## License

Apache-2.0 — see [LICENSE](./LICENSE).
