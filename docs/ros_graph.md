# ROS 2 Graph — Spider Bot Bringup

```mermaid
%%{init: {'flowchart': {'nodeSpacing': 60, 'rankSpacing': 40}}}%%
flowchart TB
    subgraph Simulation["Gazebo Harmonic (gz-sim)"]
        GZ_CTRL["gz_ros2_control\n(joint_state_broadcaster\n+ position_controller)"]
        GZ_LIDAR["Lidar sensor"]
        GZ_IMU["IMU sensor"]
        GZ_ODOM["OdometryPublisher"]
        GZ_CLOCK["/clock"]
    end

    subgraph Bridge["ros_gz_bridge"]
        BRIDGE["parameter_bridge"]
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

    subgraph Future["Planned (Nav2 Stack)"]
        RL["robot_localization\nEKF"]
        SLAM["slam_toolbox"]
        NAV2["Nav2"]
    end

    %% Gazebo topics (gz transport) -> bridge
    GZ_LIDAR -- "gz: scan" --> BRIDGE
    GZ_IMU -- "gz: imu" --> BRIDGE
    GZ_ODOM -- "gz: odom" --> BRIDGE
    GZ_ODOM -- "gz: /tf" --> BRIDGE
    GZ_CLOCK --> BRIDGE

    %% Bridge -> ROS topics (GZ_TO_ROS)
    BRIDGE --> |"/scan\nsensor_msgs/LaserScan"| SLAM
    BRIDGE --> |"/imu\nsensor_msgs/Imu"| PC
    BRIDGE --> |"/imu"| RL
    BRIDGE --> |"/odom\nnav_msgs/Odometry"| PC
    BRIDGE --> |"/odom"| RL
    BRIDGE --> |"/tf\ntf2_msgs/TFMessage"| RSP
    BRIDGE --> |"/clock\nrosgraph_msgs/Clock"| PC

    %% Joint state broadcaster
    GZ_CTRL --> JSB
    JSB --> |"/joint_states\nsensor_msgs/JointState"| PC
    JSB --> |"/joint_states"| RSP

    %% robot_state_publisher
    RSP --> |"/tf_static\ntf2_msgs/TFMessage"| PC
    RSP --> |"/robot_description\nstd_msgs/String"| SPAWN["spawn_big_bertha"]

    %% Policy controller
    PC --> |"/position_controller/commands\nstd_msgs/Float64MultiArray"| POS_CTRL
    PC --> |"policy_status\nspider_msgs/PolicyStatus"| PC

    %% cmd_vel from Nav2
    NAV2 --> |"/cmd_vel\ngeometry_msgs/Twist"| PC

    %% Services (dashed)
    PC -.-> |"set_policy_enabled\nspider_msgs/SetPolicyEnabled"| EXT1["External Client"]
    PC -.-> |"load_policy\nspider_msgs/LoadPolicy"| EXT2["External Client"]

    %% Future connections
    RL -.-> |"/odom_filtered"| NAV2
    SLAM -.-> |"/map"| NAV2

    %% Styling
    classDef node fill:#1a73e8,color:#fff,stroke:#0d47a1
    classDef topic fill:#34a853,color:#fff,stroke:#1b5e20
    classDef gz fill:#ea4335,color:#fff,stroke:#b31412
    classDef bridge fill:#fbbc04,color:#000,stroke:#ea8600
    classDef future fill:#9c27b0,color:#fff,stroke:#6a1b9a

    class PC,RSP,JSB,POS_CTRL node
    class BRIDGE bridge
    class GZ_LIDAR,GZ_IMU,GZ_ODOM,GZ_CLOCK,GZ_CTRL gz
    class RL,SLAM,NAV2 future
```

## Nodes

| Node | Package | Publisher Topics | Subscriber Topics |
|---|---|---|---|
| **policy_controller** | `big_bertha_policy_controller` | `/position_controller/commands` (Float64MultiArray), `policy_status` (PolicyStatus) | `/odom`, `/imu`, `/joint_states`, `/cmd_vel` |
| **robot_state_publisher** | `robot_state_publisher` | `/tf_static`, `/robot_description` | `/joint_states` |
| **joint_state_broadcaster** | ros2_control | `/joint_states` (sensor_msgs/JointState) | — |
| **position_controller** | ros2_control | — | `/position_controller/commands` |
| **ros_gz_bridge** | `ros_gz_bridge` | `/scan`, `/imu`, `/odom`, `/tf`, `/clock` | (gz→ros) |
| **spawn_big_bertha** | `ros_gz_sim` | — | `/robot_description` |

## Custom Interfaces

**Messages:**
- `spider_msgs/msg/PolicyStatus` — header, rate_hz, inference_ms, action_norm, enabled
- `spider_msgs/msg/GaitCommand` — vx, vy, yaw_rate

**Services:**
- `spider_msgs/srv/SetPolicyEnabled` — `bool enabled` → `bool success, string message`
- `spider_msgs/srv/LoadPolicy` — `string model_path` → `bool success, string message`
