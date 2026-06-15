#ifndef BIG_BERTHA_LEG_ODOMETRY__LEG_KINEMATICS_HPP_
#define BIG_BERTHA_LEG_ODOMETRY__LEG_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <array>

namespace big_bertha_leg_odometry {

constexpr int kNumJoints = 12;
constexpr int kNumLegs = 4;
constexpr int kJointsPerLeg = 3;

/// Geometric parameters for one leg of the Big Bertha quadruped.
///
/// Each leg has 3 continuous joints (hip yaw, hip roll, knee) and 3 rigid
/// segments (arm_a = upper, arm_b = middle, arm_c = lower/foot). The joint
/// axes and offsets are extracted from big_bertha.urdf.xacro.
struct LegGeometry {
  // Hip yaw joint: fixed offset from base_link + rotation axis.
  Eigen::Vector3d hip_offset;   // metres
  Eigen::Vector3d hip_yaw_axis; // unit vector

  // arm_a (coxa) segment: offset from hip yaw frame to hip roll joint.
  Eigen::Vector3d arm_a_offset;

  // Hip roll joint: rotation axis (45 deg in XY plane).
  Eigen::Vector3d hip_roll_axis;

  // arm_b (femur) segment: offset from hip roll frame to knee joint.
  Eigen::Vector3d arm_b_offset;

  // Knee joint: rotation axis (45 deg in XY plane).
  Eigen::Vector3d knee_axis;

  // arm_c (tibia) + foot tip: offset from knee frame to foot contact point.
  Eigen::Vector3d foot_offset;
};

/// Pre-defined geometries for the 4 legs.
///
/// Order:            index   joints   side
///   leg 0 (BL):     0-2     110-112  back-left
///   leg 1 (FL):     3-5     113-115  front-left
///   leg 2 (FR):     6-8     116-118  front-right
///   leg 3 (BR):     9-11    119-121  back-right
const std::array<LegGeometry, kNumLegs> &get_leg_geometries();

/// Compute the foot position of a given leg in the base_link frame.
///
/// @param leg_idx  Leg index [0, 4).
/// @param q        3-element joint angles for this leg [hip_yaw, hip_roll,
/// knee].
/// @return Foot tip position in base_link frame (metres).
Eigen::Vector3d compute_foot_position(int leg_idx, const Eigen::Vector3d &q);

/// Compute all 4 foot positions in base_link frame from the full 12-joint
/// state.
///
/// @param joint_pos  12-element array ordered Revolute_110 .. Revolute_121.
/// @return 4-element array of foot positions in base_link frame.
std::array<Eigen::Vector3d, kNumLegs>
compute_all_foot_positions(const std::array<double, kNumJoints> &joint_pos);

/// Compute the 3x3 linear-velocity Jacobian matrix J_v for one leg.
///
/// J_v maps joint velocities [dq_hip_yaw, dq_hip_roll, dq_knee] to the foot
/// linear velocity in base_link frame: v_foot = J_v * dq.
///
/// Uses central finite differences with a default epsilon of 1e-6 rad.
Eigen::Matrix3d compute_foot_jacobian(int leg_idx, const Eigen::Vector3d &q,
                                      double eps = 1e-6);

} // namespace big_bertha_leg_odometry

#endif // BIG_BERTHA_LEG_ODOMETRY__LEG_KINEMATICS_HPP_
