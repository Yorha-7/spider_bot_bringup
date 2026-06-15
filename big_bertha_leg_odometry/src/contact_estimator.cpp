#include "big_bertha_leg_odometry/contact_estimator.hpp"

#include <array>

#include <Eigen/Dense>

namespace big_bertha_leg_odometry {

std::array<bool, kNumLegs> estimate_contact(
    const std::array<Eigen::Vector3d, kNumLegs> &foot_positions_base,
    const Eigen::Quaterniond &imu_orientation,
    double ground_height,
    double threshold) {
  // IMU orientation quaternion rotates world -> imu_link.  To transform a
  // vector from base_link -> world we need R(q_imu).  We assume the IMU frame
  // is aligned with base_link (small offset at imu_joint is negligible).
  const Eigen::Matrix3d R_base_to_world = imu_orientation.toRotationMatrix();

  std::array<bool, kNumLegs> contact{};

  (void)ground_height;

  for (int i = 0; i < kNumLegs; ++i) {
    const Eigen::Vector3d p_base = foot_positions_base[i];
    const Eigen::Vector3d down_base = R_base_to_world.transpose() *
                                      Eigen::Vector3d::UnitZ();
    const double foot_height_along_gravity = p_base.dot(down_base);

    contact[i] = foot_height_along_gravity < threshold;
  }

  return contact;
}

}  // namespace big_bertha_leg_odometry
