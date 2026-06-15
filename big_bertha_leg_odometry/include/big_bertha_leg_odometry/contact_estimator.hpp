#ifndef BIG_BERTHA_LEG_ODOMETRY__CONTACT_ESTIMATOR_HPP_
#define BIG_BERTHA_LEG_ODOMETRY__CONTACT_ESTIMATOR_HPP_

#include <array>

#include <Eigen/Dense>

#include "big_bertha_leg_odometry/leg_kinematics.hpp"

namespace big_bertha_leg_odometry {

/// Estimate which legs are in stance (foot on ground) using world-frame
/// foot height and a tunable threshold.
///
/// A foot is classified as "stance" when its world-frame z-coordinate is
/// below a threshold (ground_height + tolerance). This assumes a flat ground
/// plane at z=0 in the odom/world frame.
///
/// @param foot_positions  Foot positions in base_link frame (4 legs).
/// @param imu_orientation IMU orientation quaternion [x, y, z, w] that rotates
///                        world -> imu_link. Used to transform foot positions
///                        into the world frame for height comparison.
/// @param ground_height   World-frame z of the ground plane (default 0.0).
/// @param threshold       Tolerance above ground for stance classification.
/// @return 4-element array where true = stance, false = swing.
std::array<bool, kNumLegs>
estimate_contact(const std::array<Eigen::Vector3d, kNumLegs> &foot_positions,
                 const Eigen::Quaterniond &imu_orientation,
                 double ground_height = 0.0, double threshold = 0.05);

} // namespace big_bertha_leg_odometry

#endif // BIG_BERTHA_LEG_ODOMETRY__CONTACT_ESTIMATOR_HPP_
