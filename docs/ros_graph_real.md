# ROS 2 Graph — Real Robot (Proposed)

```mermaid
%%{init: {'flowchart': {'nodeSpacing': 60, 'rankSpacing': 40}}}%%
flowchart TB
    subgraph Hardware["Real Robot Hardware"]
        MG995["MG995 Servos ×12\n(4 legs × 3 joints)"]
        YDLIDAR["YDLidar X2"]
        MPU6050["MPU6050 IMU"]
    end

    subgraph Drivers["Hardware Driver Nodes"]
        SERVO_BRIDGE["servo_bridge\n(C++ serial/PWM)\n→ MG995 servos"]
        LIDAR_DRIVER["ydlidar_ros2_driver\n→ /scan"]
        IMU_DRIVER["imu_driver_node\n(C++) → /imu"]
        HW_IF["ros2_control Hardware Interface\n(RobotHW)\nreads /position_controller/commands\nwrites /joint_states"]
    end

    subgraph State["Robot State"]
        RSP["robot_state_publisher"]
        JSB["joint_state_broadcaster"]
    end

    subgraph Control["ros2_control"]
        POS_CTRL["position_controller\n(forward_command_controller)"]
    end

    subgraph Policy["Gait Controller"]
        PC["policy_controller\n(C++ ONNX Runtime)"]
    end

    subgraph Nav["Navigation Stack"]
        EKF["robot_localization\nEKF"]
        SLAM["slam_toolbox"]
        AMCL["AMCL + map_server"]
        NAV2["Nav2\nplanner+controller+costmaps+BT"]
    end

    subgraph Viz["Visualization"]
        RVIZ["RViz2"]
    end

    %% Hardware → Drivers
    MG995 --- SERVO_BRIDGE
    YDLIDAR --- LIDAR_DRIVER
    MPU6050 --- IMU_DRIVER

    %% Joint state feedback
    HW_IF --> JSB
    JSB --> |"/joint_states\nsensor_msgs/JointState"| PC
    JSB --> RSP

    %% LIDAR
    LIDAR_DRIVER --> |"/scan\nsensor_msgs/LaserScan"| SLAM
    LIDAR_DRIVER --> |"/scan"| NAV2

    %% IMU
    IMU_DRIVER --> |"/imu\nsensor_msgs/Imu"| PC
    IMU_DRIVER --> |"/imu"| EKF

    %% Odometry (leg odometry from joint states → EKF)
    PC -.-> |"leg_odom\nnav_msgs/Odometry\n(proposed)"| EKF

    %% robot_state_publisher
    RSP --> |"/tf_static\ntf2_msgs/TFMessage"| PC
    RSP --> |"/robot_description"| RVIZ

    %% Policy controller
    PC --> |"/position_controller/commands\nstd_msgs/Float64MultiArray"| POS_CTRL
    POS_CTRL --> HW_IF
    PC --> |"policy_status\nspider_msgs/PolicyStatus"| RVIZ

    %% Services
    PC -.-> |"set_policy_enabled\nspider_msgs/SetPolicyEnabled"| CLI["ros2 CLI / RQT"]
    PC -.-> |"load_policy\nspider_msgs/LoadPolicy"| CLI

    %% Navigation stack
    EKF --> |"odom→base_link tf"| SLAM
    EKF --> |"odom→base_link tf"| AMCL
    SLAM --> |"/map + map→odom tf"| NAV2
    AMCL --> |"map→odom tf"| NAV2
    NAV2 --> |"/cmd_vel\ngeometry_msgs/Twist"| PC

    %% RViz
    NAV2 --> RVIZ
    SLAM --> RVIZ

    %% Styling
    classDef hardware fill:#e8f5e9,color:#1b5e20,stroke:#2e7d32
    classDef driver fill:#c8e6c9,color:#1b5e20,stroke:#388e3c
    classDef node fill:#1a73e8,color:#fff,stroke:#0d47a1
    classDef control fill:#1565c0,color:#fff,stroke:#0d47a1
    classDef nav fill:#9c27b0,color:#fff,stroke:#6a1b9a
    classDef viz fill:#f57f17,color:#fff,stroke:#e65100

    class MG995,YDLIDAR,MPU6050 hardware
    class SERVO_BRIDGE,LIDAR_DRIVER,IMU_DRIVER,HW_IF driver
    class PC,RSP,JSB node
    class POS_CTRL control
    class EKF,SLAM,AMCL,NAV2 nav
    class RVIZ,CLI viz
```

## Nodes

| Node | Package | Publisher Topics | Subscriber Topics |
|---|---|---|---|
| **policy_controller** | `big_bertha_policy_controller` | `/position_controller/commands` (Float64MultiArray), `policy_status` (PolicyStatus), `leg_odom` (Odometry, proposed) | `/odom`, `/imu`, `/joint_states`, `/cmd_vel` |
| **robot_state_publisher** | `robot_state_publisher` | `/tf_static`, `/robot_description` | `/joint_states` |
| **joint_state_broadcaster** | ros2_control | `/joint_states` (sensor_msgs/JointState) | — |
| **position_controller** | ros2_control | — | `/position_controller/commands` |
| **servo_bridge** | `big_bertha_bringup` (TODO) | — | internal: position targets → serial/PWM |
| **imu_driver_node** | `big_bertha_bringup` (TODO) | `/imu` (sensor_msgs/Imu) | — |
| **ydlidar_ros2_driver** | `ydlidar_ros2_driver` | `/scan` (sensor_msgs/LaserScan) | — |
| **robot_localization** | `robot_localization` | filtered odometry + tf | `/imu`, `/odom` |

## Sim → Real Changes

| Sim | Real Robot |
|---|---|
| Gazebo Harmonic (physics engine) | Physical MG995 servos + sensors |
| `ros_gz_bridge` | `ydlidar_ros2_driver`, `imu_driver_node`, `servo_bridge` |
| `gz_ros2_control` (Gazebo plugin) | Custom `RobotHW` ros2_control interface → serial/PWM |
| `/odom` from Gazebo OdometryPublisher | Leg odometry from joint states → EKF (`leg_odom` proposed) |
| `use_sim_time:=true` | `use_sim_time:=false` |

## Hardware BOM (from PLAN.md §11)

- **Compute:** Arduino UNO Q (4 GB, arm64) — ROS 2 Jazzy
- **Actuators:** 12× MG995 servos (4 legs × 3 joints)
- **Lidar:** 1× YDLidar X2
- **IMU:** 1× MPU6050
