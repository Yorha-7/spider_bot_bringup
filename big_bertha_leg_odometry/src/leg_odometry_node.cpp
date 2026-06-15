#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "big_bertha_leg_odometry/contact_estimator.hpp"
#include "big_bertha_leg_odometry/leg_kinematics.hpp"

using namespace std::chrono_literals;
namespace bblo = big_bertha_leg_odometry;

class LegOdometryNode : public rclcpp::Node {
public:
  LegOdometryNode() : Node("leg_odometry_node") {
    // ----------------------------- Parameters ----------------------------
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    ground_height_ = declare_parameter<double>("ground_height", 0.0);
    stance_threshold_ = declare_parameter<double>("stance_threshold", 0.03);
    covariance_scale_ = declare_parameter<double>("covariance_scale", 0.01);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"Revolute_110", "Revolute_111", "Revolute_112", "Revolute_113",
         "Revolute_114", "Revolute_115", "Revolute_116", "Revolute_117",
         "Revolute_118", "Revolute_119", "Revolute_120", "Revolute_121"});

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = static_cast<int>(i);
    }

    // --------------------------- Pub / Sub ------------------------------
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/leg_odom", rclcpp::QoS(1));
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&LegOdometryNode::on_joints, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        std::bind(&LegOdometryNode::on_imu, this, std::placeholders::_1));

    // ----------------------------- Timer --------------------------------
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LegOdometryNode::control_loop, this));

    RCLCPP_INFO(get_logger(),
                "leg_odometry_node up: rate=%.1f Hz, frame=%s",
                publish_rate_, odom_frame_.c_str());
  }

private:
  void on_joints(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = joint_index_.find(msg->name[i]);
      if (it == joint_index_.end()) continue;
      const int idx = it->second;
      if (i < msg->position.size()) joint_pos_[idx] = msg->position[i];
      if (i < msg->velocity.size()) joint_vel_[idx] = msg->velocity[i];
    }
    joint_stamp_ = msg->header.stamp;
    have_joints_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    imu_orientation_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z);
    imu_ang_vel_ = Eigen::Vector3d(
        msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z);
    have_imu_ = true;
  }

  void control_loop() {
    if (!have_joints_) return;

    // Snapshot joint state.
    std::array<double, bblo::kNumJoints> pos, vel;
    Eigen::Quaterniond imu_orientation(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d imu_omega = Eigen::Vector3d::Zero();
    bool use_imu;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      pos = joint_pos_;
      vel = joint_vel_;
      stamp = joint_stamp_;
      use_imu = have_imu_;
      if (use_imu) {
        imu_orientation = imu_orientation_;
        imu_omega = imu_ang_vel_;
      }
    }

    // Sanitize joint data.
    for (auto &v : pos) {
      if (!std::isfinite(v)) v = 0.0;
    }
    for (auto &v : vel) {
      if (!std::isfinite(v)) v = 0.0;
    }

    // 1. Forward kinematics: compute foot positions in base frame.
    const auto foot_positions = bblo::compute_all_foot_positions(pos);

    // 2. Estimate contact: which feet are on the ground?
    const auto contact = bblo::estimate_contact(
        foot_positions, imu_orientation, ground_height_, stance_threshold_);

    // 3. Compute foot velocities from Jacobian: v_foot_base = J(q) * dq.
    std::array<Eigen::Vector3d, bblo::kNumLegs> foot_vel_base;
    for (int leg = 0; leg < bblo::kNumLegs; ++leg) {
      const int base = leg * bblo::kJointsPerLeg;
      Eigen::Vector3d q(pos[base], pos[base + 1], pos[base + 2]);
      Eigen::Vector3d dq(vel[base], vel[base + 1], vel[base + 2]);
      Eigen::Matrix3d J = bblo::compute_foot_jacobian(leg, q);
      foot_vel_base[leg] = J * dq;
    }

    // 4. Estimate body twist from stance feet.
    //
    // For each stance foot: v_body + omega_body × p_foot = -v_foot_base
    //
    // If IMU is available, use its angular velocity directly and solve only
    // for linear velocity (3-DOF, one stance foot is enough).  Otherwise
    // solve the full 6-DOF system (requires >= 2 stance feet).
    //
    // Method A (IMU): v_body = -omega_imu × p_foot - v_foot_base
    //   Stack all stance legs and average.
    //
    // Method B (no IMU): solve least squares for [v_body; omega_body].

    Eigen::Vector3d v_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();
    int stance_count = 0;

    if (use_imu) {
      omega_body = imu_omega;
      Eigen::Vector3d sum_v = Eigen::Vector3d::Zero();
      for (int leg = 0; leg < bblo::kNumLegs; ++leg) {
        if (!contact[leg]) continue;
        sum_v += -omega_body.cross(foot_positions[leg]) - foot_vel_base[leg];
        ++stance_count;
      }
      if (stance_count > 0) {
        v_body = sum_v / static_cast<double>(stance_count);
      }
    } else {
      // Full 6-DOF least squares: A (3n × 6) * x (6) = b (3n)
      // x = [vx, vy, vz, wx, wy, wz]^T
      // A_i = [I_3, -skew(p_foot_i)]
      // b_i = -v_foot_base_i
      Eigen::MatrixXd A(3 * bblo::kNumLegs, 6);
      Eigen::VectorXd b(3 * bblo::kNumLegs);
      int row = 0;
      for (int leg = 0; leg < bblo::kNumLegs; ++leg) {
        if (!contact[leg]) continue;
        const Eigen::Vector3d p = foot_positions[leg];
        A.block<3, 3>(row, 0) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(row, 3) = -skew_symmetric(p);
        b.segment<3>(row) = -foot_vel_base[leg];
        row += 3;
        ++stance_count;
      }
      if (row >= 6) {
        A.conservativeResize(row, Eigen::NoChange);
        b.conservativeResize(row);
        Eigen::Matrix<double, 6, 1> x =
            (A.transpose() * A).ldlt().solve(A.transpose() * b);
        v_body = x.head<3>();
        omega_body = x.tail<3>();
      } else if (row > 0) {
        A.conservativeResize(row, Eigen::NoChange);
        b.conservativeResize(row);
        Eigen::Matrix<double, 6, 1> x =
            A.transpose() * (A * A.transpose()).ldlt().solve(b);
        v_body = x.head<3>();
        omega_body = x.tail<3>();
      }
    }

    // Sanitize outputs.
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(v_body[i])) v_body[i] = 0.0;
      if (!std::isfinite(omega_body[i])) omega_body[i] = 0.0;
    }

    // 5. Integrate pose (dead-reckoning from twist).
    const rclcpp::Time now_time = now();
    if (last_time_.nanoseconds() == 0) {
      last_time_ = now_time;
    }
    const double dt = (now_time - last_time_).seconds();
    last_time_ = now_time;

    if (dt > 0.0 && dt < 0.1) {
      const Eigen::Matrix3d R_body_to_world =
          imu_orientation.toRotationMatrix();
      base_position_ += R_body_to_world * v_body * dt;
    }
    base_orientation_ = imu_orientation;

    // 6. Build odometry message.
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Pose.
    odom.pose.pose.position.x = base_position_.x();
    odom.pose.pose.position.y = base_position_.y();
    odom.pose.pose.position.z = base_position_.z();
    odom.pose.pose.orientation.x = base_orientation_.x();
    odom.pose.pose.orientation.y = base_orientation_.y();
    odom.pose.pose.orientation.z = base_orientation_.z();
    odom.pose.pose.orientation.w = base_orientation_.w();

    // Pose covariance (diagonal): scale inversely with stance count.
    const double pc = stance_count > 0
                          ? covariance_scale_ / stance_count
                          : covariance_scale_;
    auto &pcov = odom.pose.covariance;
    pcov[0] = pcov[7] = pcov[14] = pc;        // x, y, z
    pcov[21] = pcov[28] = pcov[35] = 0.01;    // roll, pitch, yaw

    // Twist (body frame).
    odom.twist.twist.linear.x = v_body.x();
    odom.twist.twist.linear.y = v_body.y();
    odom.twist.twist.linear.z = v_body.z();
    odom.twist.twist.angular.x = omega_body.x();
    odom.twist.twist.angular.y = omega_body.y();
    odom.twist.twist.angular.z = omega_body.z();

    // Twist covariance.
    auto &tcov = odom.twist.covariance;
    tcov[0] = tcov[7] = tcov[14] = pc;        // linear
    tcov[21] = tcov[28] = tcov[35] = 0.01;    // angular

    odom_pub_->publish(odom);

    // 7. Broadcast tf: odom -> base_link.
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = base_position_.x();
      tf.transform.translation.y = base_position_.y();
      tf.transform.translation.z = base_position_.z();
      tf.transform.rotation.x = base_orientation_.x();
      tf.transform.rotation.y = base_orientation_.y();
      tf.transform.rotation.z = base_orientation_.z();
      tf.transform.rotation.w = base_orientation_.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }

  static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return S;
  }

  // State (mutex-protected between callbacks and control loop).
  std::mutex state_mutex_;
  std::array<double, bblo::kNumJoints> joint_pos_{};
  std::array<double, bblo::kNumJoints> joint_vel_{};
  Eigen::Quaterniond imu_orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d imu_ang_vel_{0.0, 0.0, 0.0};
  bool have_joints_{false};
  bool have_imu_{false};
  rclcpp::Time joint_stamp_{0, 0, RCL_ROS_TIME};

  // Pose integration (dead-reckoning).
  Eigen::Vector3d base_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond base_orientation_{1.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  // Parameters.
  double publish_rate_{50.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double ground_height_{0.0};
  double stance_threshold_{0.03};
  double covariance_scale_{0.01};
  bool publish_tf_{true};
  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_index_;

  // ROS.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
