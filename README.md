# Big Bertha Bringup

ROS 2 **Jazzy** bringup for the **Big Bertha** quadruped — a PPO locomotion policy
(exported to ONNX, run from a C++ node) drives a learned gait, while SLAM builds a
map and Nav2 plans collision-free paths. Demonstrated in Gazebo Harmonic with a
full obstacle course. Real-hardware support is scaffolded for Arduino UNO Q + 12
MG995 servos + MPU6050 IMU + YDLidar X2.

> **Simulation first** — everything works in Gazebo today.
> Hardware drivers exist but the full real-robot autonomy stack is a work in progress.
> See **[PLAN.md](./PLAN.md)** for the full architecture, module decomposition, and execution plan.

---

## Guided tour

### 1. Prerequisites

- **Ubuntu 24.04** (for ROS 2 Jazzy + Gazebo Harmonic)
- No hardware needed — the entire tour runs in simulation

### 2. Install & build

```bash
# One-shot environment install (ROS 2 Jazzy, Gazebo Harmonic, Nav2, ros2_control, tooling)
bash scripts/install_jazzy.sh

# Source ROS 2 (or open a new terminal)
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build --symlink-install && source install/setup.bash
```

### 3. Meet the robot model

```bash
ros2 launch big_bertha_description rsp.launch.py use_sim_time:=false
```

This launches `robot_state_publisher` from the URDF in
[`big_bertha_description`](./big_bertha_description). Open RViz and add a RobotModel
display to see Big Bertha's four three-jointed legs, lidar, IMU, and battery link.

File layout:
- `urdf/big_bertha.urdf.xacro` — main model (12 continuous joints, 4 legs)
- `urdf/big_bertha.ros2_control.xacro` — `gz_ros2_control` system plugin
- `urdf/big_bertha.gazebo.xacro` — Gazebo sensors (GPU lidar, IMU, odometry pub)
- `meshes/` — 15 STL files (base_link, battery, ydlidar, arm segments)
- `config/ros2_control.yaml` — controller manager config (200 Hz update rate)

### 4. Start the simulation

```bash
ros2 launch big_bertha_sim_bringup simulation.launch.py gui:=true
```

This does the following:
- Starts **Gazebo Harmonic** with the `obstacle_world.sdf` (10×10 m arena with
  walls, boxes, pillars — spawn at A, goal at B)
- Spawns Big Bertha at `(-3.5, -3.5, 0.12)`
- Runs `robot_state_publisher` for TF
- Launches the **ros_gz_bridge** to relay Gazebo topics to ROS 2:
  `/clock`, `/scan`, `/imu`, `/odom`, `/tf`
- Spawns the `joint_state_broadcaster` and `position_controller`

After a few seconds you should see Big Bertha standing in the obstacle course.
In a separate terminal, inspect the active ROS 2 graph:

```bash
rqt_graph
```

You'll see nodes for `robot_state_publisher`, `joint_state_broadcaster`,
`position_controller`, the bridge, and the Gazebo server — all wired together.

### 5. Bring the gait to life

```bash
ros2 launch big_bertha_policy_controller policy_controller.launch.py
```

The **policy controller** ([`big_bertha_policy_controller`](./big_bertha_policy_controller))
is a C++ node that runs the trained PPO policy via ONNX Runtime. It subscribes to:

| Topic | Type | Used for |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | body linear velocity |
| `/imu` | `sensor_msgs/Imu` | angular velocity + orientation (gravity vector) |
| `/joint_states` | `sensor_msgs/JointState` | joint positions + velocities |
| `/cmd_vel` | `geometry_msgs/Twist` | commanded velocity (from Nav2 or teleop) |

It builds a **48-element observation vector** and runs the ONNX model at **50 Hz**:

```
[0:3]   root_lin_vel_b        from /odom
[3:6]   root_ang_vel_b        from /imu
[6:9]   projected_gravity_b   from /imu orientation
[9:12]  commands vx, vy, yaw  from /cmd_vel
[12:24] joint_pos - default    from /joint_states (12 joints)
[24:36] joint_vel              from /joint_states (12 joints)
[36:48] prev_actions           previous 12 joint targets
```

Output: 12 joint targets published to `/position_controller/commands`
(`std_msgs/Float64MultiArray`), which Gazebo's `position_controller` actuates.

Watch the gait in action:

```bash
ros2 topic echo /position_controller/commands
```

You'll see 12 floating-point joint targets streaming at 50 Hz.

The node also publishes `policy_status` (`spider_msgs/PolicyStatus`) for
introspection and exposes two services:

```bash
# Disable the gait (servos go limp)
ros2 service call /set_policy_enabled spider_msgs/srv/SetPolicyEnabled "{enabled: false}"

# Re-enable
ros2 service call /set_policy_enabled spider_msgs/srv/SetPolicyEnabled "{enabled: true}"

# Hot-swap the ONNX model
ros2 service call /load_policy spider_msgs/srv/LoadPolicy "{model_path: '/path/to/new_policy.onnx'}"
```

### 6. Drive it around

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press `i` (forward), `,` (backward), `j`/`l` (turn), `u`/`o` (strafe).
The policy controller receives `/cmd_vel`, the observation vector changes, and
the gait adapts in real time. If you stop sending commands for more than 0.5 s,
the controller zeros out the command (safety timeout).

### 7. Leg odometry & state estimation

Leg odometry computes base twist from joint kinematics alone — no lidar or
camera needed. Start it alongside the simulation:

```bash
ros2 launch big_bertha_leg_odometry leg_odometry.launch.py
```

The algorithm in [`big_bertha_leg_odometry`](./big_bertha_leg_odometry):
1. Forward kinematics: foot positions from 12 joint angles
2. Contact estimation: stance vs. swing from world-frame foot height
3. Body twist via least-squares from stance-foot velocities
4. Dead-reckoning pose integration → `/leg_odom`

Fuse it with the IMU using the EKF:

```bash
ros2 launch big_bertha_state_estimation state_estimation/ekf.launch.py
```

The [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization) EKF
takes `/leg_odom` (differential twist) + `/imu` (orientation + angular velocity)
and publishes a smoothed `/odom_filtered` + `odom→base_link` transform.

**Sim vs. real:** In simulation, Gazebo's ground-truth `/odom` replaces `/leg_odom`.
The EKF configs are pre-configured for both modes:
- `config/ekf.yaml` — real robot (fuses `/leg_odom` + `/imu`)
- `config/ekf_sim.yaml` — simulation (fuses `/odom` + `/imu`)

### 8. Mapping, localization & planning (future)

The next modules are scaffolded but not yet implemented:

| Module | Package | Status |
|---|---|---|
| Mapping | `slam_toolbox` | ❌ Not started |
| Localization | AMCL + `map_server` | ❌ Not started |
| Planning | Nav2 (planner, controller, costmaps, BT) | ❌ Not started |
| Integration | Full `bringup.launch.py` with RViz | ❌ Not started |

Once complete, the data flow will be:

```
Nav2 → /cmd_vel → policy_controller → /position_controller/commands → Gazebo
Gazebo → /joint_states, /odom, /imu → policy_controller (observation)
Gazebo → /scan → slam_toolbox → /map → Nav2
Gazebo → /odom, /imu → robot_localization EKF → /odom_filtered → Nav2
```

---

## Hardware tour

The hardware bridge connects Big Bertha's physical components to the same ROS 2 stack.

### Components

| Part | Qty | Interface |
|---|---|---|
| Arduino UNO Q (arm64, 4 GB) | 1 | ROS 2 Jazzy onboard |
| MG995 servo | 12 | PCA9685 PWM driver (I2C addr 0x40) |
| MPU6050 IMU | 1 | I2C (addr 0x68) |
| YDLidar X2 | 1 | USB serial |

### Firmware

The firmware lives in [`big_bertha_hardware_bringup/firmware/`](./big_bertha_hardware_bringup/firmware/)
— an STM32 App Lab project (Arduino UNO Q). It uses register-level I2C (no
third-party libraries) to:
- Read MPU6050 gyro + accel at 200 Hz
- Drive 12 MG995 servos via PCA9685 at 50 Hz (500–2500 µs pulses)

### ROS 2 bridge node

```bash
ros2 launch big_bertha_hardware_bringup hardware_bridge.launch.py
```

The Python bridge node ([`hardware_bridge_node.py`](./big_bertha_hardware_bringup/big_bertha_hardware_bringup/hardware_bridge_node.py))
communicates with the STM32 firmware over a Unix socket (msgpack RPC):

| Direction | Call | Payload |
|---|---|---|
| ← ROS | `read_imu` | gyro (rad/s) + accel (m/s²) → `/imu` at 30 Hz |
| → ROS | `set_servos` | `/position_controller/commands` → 12 PWM pulses |
| ↔ | `ping` | health check |

Use `mock:=true` for testing without hardware:

```bash
ros2 launch big_bertha_hardware_bringup hardware_bridge.launch.py mock:=true
```

### Hardware bringup order

1. ✅ MPU6050 + PCA9685 driver (`hardware_bridge`)
2. ✅ Leg odometry — `/leg_odom` from joint kinematics
3. ❌ YDLidar X2 — `ydlidar_ros2_driver` → `/scan`
4. ✅ State estimation — `robot_localization` EKF

---

## Package reference

| Package | Type | Role |
|---|---|---|
| [`spider_msgs`](./spider_msgs) | `ament_cmake` | Robot-agnostic interfaces (shared with future Lil Spider) — `PolicyStatus`, `GaitCommand`, `SetPolicyEnabled`, `LoadPolicy` |
| [`big_bertha_description`](./big_bertha_description) | `ament_cmake` | URDF/xacro, meshes, `ros2_control` + Gazebo plugin config |
| [`big_bertha_policy_controller`](./big_bertha_policy_controller) | `ament_cmake` (C++) | ONNX gait node — 48-obs PPO policy → 12 joint targets at 50 Hz |
| [`big_bertha_leg_odometry`](./big_bertha_leg_odometry) | `ament_cmake` (C++) | Forward-kinematics leg odometry — `/leg_odom` from joint states + IMU |
| [`big_bertha_state_estimation`](./big_bertha_state_estimation) | `ament_cmake` | `robot_localization` EKF config + launch — fuses `/leg_odom` + `/imu` |
| [`big_bertha_sim_bringup`](./big_bertha_sim_bringup) | `ament_cmake` | Gazebo simulation orchestrator — world, robot spawn, bridge, controllers |
| [`big_bertha_hardware_bringup`](./big_bertha_hardware_bringup) | `ament_python` | Hardware bridge node + STM32 firmware (MPU6050 + PCA9685) |

### Dependency graph

```
spider_msgs  (custom interfaces)
    ↑
big_bertha_description  (URDF, meshes, ros2_control config)
    ↑
big_bertha_policy_controller  (C++ ONNX gait, depends on spider_msgs)
    ↑
big_bertha_leg_odometry  (C++ FK-based odometry)
    ↑
big_bertha_state_estimation  (robot_localization EKF)
    ↑
big_bertha_sim_bringup  (Gazebo orchestrator, pulls in everything above)
    │
big_bertha_hardware_bringup  (standalone — real-robot counterpart)
```

### ROS graph diagrams

Visual node/topic maps are in [`docs/`](./docs/):
- [`ros_graph.md`](./docs/ros_graph.md) — simulation graph (Mermaid)
- [`ros_graph_real.md`](./docs/ros_graph_real.md) — proposed real-robot graph

---

## Development

See **[CONTRIBUTING.md](./CONTRIBUTING.md)** for:
- Build & test workflow (colcon)
- Code style (C++17 Google style, ament_flake8 + pep257 for Python)
- Commit conventions (commitlint)
- PR guidelines (one module per PR, CI must pass)

### Quick reference

```bash
# Build
colcon build --symlink-install && source install/setup.bash

# Run tests
colcon test && colcon test-result --verbose

# Lint
python3 -m ament_flake8 big_bertha_hardware_bringup
```

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
| 6 | Leg odometry | ✅ Done |
| 7 | State estimation (robot_localization EKF) | ✅ Done |
| 8 | Mapping (slam_toolbox) | ❌ Not started |
| 9 | Localization (AMCL + map_server) | ❌ Not started |
| 10 | Planning (Nav2) | ❌ Not started |
| 11 | Integration + end-to-end verification | ❌ Not started |

---

## License

Apache-2.0 — see [LICENSE](./LICENSE).
