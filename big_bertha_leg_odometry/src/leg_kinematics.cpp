#include "big_bertha_leg_odometry/leg_kinematics.hpp"

#include <array>
#include <cmath>

#include <Eigen/Dense>

namespace big_bertha_leg_odometry {

// ---------------------------------------------------------------------------
// Leg geometry definitions
//
// Extracted from big_bertha.urdf.xacro:
//   joint |  origin xyz (m)              |  axis
//   ------+------------------------------+-------------------------
//   110   | -0.0802, -0.0649,  0.0278   |  0,          0,          1
//   111   | -0.015981, -0.055296, -0.0141 |  0.707107,  -0.707107,  0
//   112   | -0.028284, -0.028284,  0.069282 | -0.707107,  0.707107,  0
//   113   |  0.0802, -0.0649,  0.0278   |  0,          0,          1
//   114   |  0.055296, -0.015981, -0.0141 |  0.707107,   0.707107,  0
//   115   |  0.028284, -0.028284,  0.069282 | -0.707107, -0.707107,  0
//   116   |  0.0802,  0.0649,  0.0278   |  0,          0,          1
//   117   |  0.015981,  0.055296, -0.0141 | -0.707107,  0.707107,  0
//   118   |  0.028284,  0.028284,  0.069282 |  0.707107, -0.707107,  0
//   119   | -0.0802,  0.0649,  0.0278   |  0,          0,          1
//   120   | -0.055296,  0.015981, -0.0141 | -0.707107, -0.707107,  0
//   121   | -0.028284,  0.028284,  0.069282 |  0.707107,  0.707107,  0
// ---------------------------------------------------------------------------

static LegGeometry make_leg_0_geometry() {  // BL (back-left)
  LegGeometry g;
  g.hip_offset     = Eigen::Vector3d(-0.0802, -0.0649, 0.0278);
  g.hip_yaw_axis   = Eigen::Vector3d::UnitZ();
  g.arm_a_offset   = Eigen::Vector3d(-0.015981, -0.055296, -0.0141);
  g.hip_roll_axis  = Eigen::Vector3d(0.707107, -0.707107, 0.0).normalized();
  g.arm_b_offset   = Eigen::Vector3d(-0.028284, -0.028284, 0.069282);
  g.knee_axis      = Eigen::Vector3d(-0.707107, 0.707107, 0.0).normalized();
  g.foot_offset    = Eigen::Vector3d::Zero();
  return g;
}

static LegGeometry make_leg_1_geometry() {  // FL (front-left)
  LegGeometry g;
  g.hip_offset     = Eigen::Vector3d(0.0802, -0.0649, 0.0278);
  g.hip_yaw_axis   = Eigen::Vector3d::UnitZ();
  g.arm_a_offset   = Eigen::Vector3d(0.055296, -0.015981, -0.0141);
  g.hip_roll_axis  = Eigen::Vector3d(0.707107, 0.707107, 0.0).normalized();
  g.arm_b_offset   = Eigen::Vector3d(0.028284, -0.028284, 0.069282);
  g.knee_axis      = Eigen::Vector3d(-0.707107, -0.707107, 0.0).normalized();
  g.foot_offset    = Eigen::Vector3d::Zero();
  return g;
}

static LegGeometry make_leg_2_geometry() {  // FR (front-right)
  LegGeometry g;
  g.hip_offset     = Eigen::Vector3d(0.0802, 0.0649, 0.0278);
  g.hip_yaw_axis   = Eigen::Vector3d::UnitZ();
  g.arm_a_offset   = Eigen::Vector3d(0.015981, 0.055296, -0.0141);
  g.hip_roll_axis  = Eigen::Vector3d(-0.707107, 0.707107, 0.0).normalized();
  g.arm_b_offset   = Eigen::Vector3d(0.028284, 0.028284, 0.069282);
  g.knee_axis      = Eigen::Vector3d(0.707107, -0.707107, 0.0).normalized();
  g.foot_offset    = Eigen::Vector3d::Zero();
  return g;
}

static LegGeometry make_leg_3_geometry() {  // BR (back-right)
  LegGeometry g;
  g.hip_offset     = Eigen::Vector3d(-0.0802, 0.0649, 0.0278);
  g.hip_yaw_axis   = Eigen::Vector3d::UnitZ();
  g.arm_a_offset   = Eigen::Vector3d(-0.055296, 0.015981, -0.0141);
  g.hip_roll_axis  = Eigen::Vector3d(-0.707107, -0.707107, 0.0).normalized();
  g.arm_b_offset   = Eigen::Vector3d(-0.028284, 0.028284, 0.069282);
  g.knee_axis      = Eigen::Vector3d(0.707107, 0.707107, 0.0).normalized();
  g.foot_offset    = Eigen::Vector3d::Zero();
  return g;
}

const std::array<LegGeometry, kNumLegs> &get_leg_geometries() {
  static const std::array<LegGeometry, kNumLegs> kGeometries = {
      make_leg_0_geometry(),
      make_leg_1_geometry(),
      make_leg_2_geometry(),
      make_leg_3_geometry(),
  };
  return kGeometries;
}

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

Eigen::Vector3d compute_foot_position(int leg_idx, const Eigen::Vector3d &q) {
  const auto &g = get_leg_geometries()[leg_idx];

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(g.hip_offset);
  T.rotate(Eigen::AngleAxisd(q[0], g.hip_yaw_axis));
  T.translate(g.arm_a_offset);
  T.rotate(Eigen::AngleAxisd(q[1], g.hip_roll_axis));
  T.translate(g.arm_b_offset);
  T.rotate(Eigen::AngleAxisd(q[2], g.knee_axis));
  T.translate(g.foot_offset);
  return T.translation();
}

std::array<Eigen::Vector3d, kNumLegs>
compute_all_foot_positions(const std::array<double, kNumJoints> &joint_pos) {
  std::array<Eigen::Vector3d, kNumLegs> feet;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const int base = leg * kJointsPerLeg;
    feet[leg] = compute_foot_position(
        leg, Eigen::Vector3d(joint_pos[base + 0],
                             joint_pos[base + 1],
                             joint_pos[base + 2]));
  }
  return feet;
}

// ---------------------------------------------------------------------------
// Finite-difference Jacobian
// ---------------------------------------------------------------------------

Eigen::Matrix3d compute_foot_jacobian(int leg_idx,
                                       const Eigen::Vector3d &q,
                                       double eps) {
  Eigen::Matrix3d J;
  for (int j = 0; j < kJointsPerLeg; ++j) {
    Eigen::Vector3d q_plus = q;
    Eigen::Vector3d q_minus = q;
    q_plus[j] += eps;
    q_minus[j] -= eps;
    Eigen::Vector3d p_plus = compute_foot_position(leg_idx, q_plus);
    Eigen::Vector3d p_minus = compute_foot_position(leg_idx, q_minus);
    J.col(j) = (p_plus - p_minus) / (2.0 * eps);
  }
  return J;
}

}  // namespace big_bertha_leg_odometry
