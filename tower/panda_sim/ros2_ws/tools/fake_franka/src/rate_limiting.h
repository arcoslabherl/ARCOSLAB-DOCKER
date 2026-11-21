// libfranka 0.8.0's rate limiter for joint positions, as the real library
// applies it to every command when control() is called with limit_rate = true
// (src/control_loop.cpp:201):
//   q_c = limitRate(kMaxJointVelocity, kMaxJointAcceleration, kMaxJointJerk,
//                   q_c, robot_state.q_d, robot_state.dq_d, robot_state.ddq_d);
// Constants and functions copied from libfranka 0.8.0 (faa0219),
// include/franka/rate_limiting.h and src/rate_limiting.cpp, only the joint
// position path. Copyright (c) 2017 Franka Emika GmbH, Apache-2.0.
#pragma once

#include <algorithm>
#include <array>

namespace franka {
namespace libfranka_080 {

constexpr double kDeltaT = 1e-3;
constexpr double kLimitEps = 1e-3;
// "When a packet is lost, FCI assumes a constant acceleration model"
constexpr double kTolNumberPacketsLost = 3.0;

constexpr std::array<double, 7> kMaxJointJerk{
    {7500.0 - kLimitEps, 3750.0 - kLimitEps, 5000.0 - kLimitEps, 6250.0 - kLimitEps,
     7500.0 - kLimitEps, 10000.0 - kLimitEps, 10000.0 - kLimitEps}};
constexpr std::array<double, 7> kMaxJointAcceleration{
    {15.0000 - kLimitEps, 7.500 - kLimitEps, 10.0000 - kLimitEps, 12.5000 - kLimitEps,
     15.0000 - kLimitEps, 20.0000 - kLimitEps, 20.0000 - kLimitEps}};
constexpr std::array<double, 7> kMaxJointVelocity{
    {2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[0],
     2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[1],
     2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[2],
     2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[3],
     2.6100 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[4],
     2.6100 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[5],
     2.6100 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[6]}};

inline double limitRate(double max_velocity,
                        double max_acceleration,
                        double max_jerk,
                        double commanded_velocity,
                        double last_commanded_velocity,
                        double last_commanded_acceleration) {
  // Differentiate to get jerk
  double commanded_jerk =
      (((commanded_velocity - last_commanded_velocity) / kDeltaT) - last_commanded_acceleration) /
      kDeltaT;

  // Limit jerk and integrate to get acceleration
  double commanded_acceleration = last_commanded_acceleration +
                                  std::max(std::min(commanded_jerk, max_jerk), -max_jerk) * kDeltaT;

  // Compute acceleration limits
  double safe_max_acceleration = std::min(
      (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity), max_acceleration);
  double safe_min_acceleration = std::max(
      (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity), -max_acceleration);

  // Limit acceleration and integrate to get desired velocities
  return last_commanded_velocity +
         std::max(std::min(commanded_acceleration, safe_max_acceleration), safe_min_acceleration) *
             kDeltaT;
}

inline double limitRate(double max_velocity,
                        double max_acceleration,
                        double max_jerk,
                        double commanded_position,
                        double last_commanded_position,
                        double last_commanded_velocity,
                        double last_commanded_acceleration) {
  return last_commanded_position +
         limitRate(max_velocity, max_acceleration, max_jerk,
                   (commanded_position - last_commanded_position) / kDeltaT,
                   last_commanded_velocity, last_commanded_acceleration) *
             kDeltaT;
}

inline std::array<double, 7> limitRate(const std::array<double, 7>& commanded_positions,
                                       const std::array<double, 7>& last_commanded_positions,
                                       const std::array<double, 7>& last_commanded_velocities,
                                       const std::array<double, 7>& last_commanded_accelerations) {
  std::array<double, 7> limited{};
  for (size_t i = 0; i < 7; i++) {
    limited[i] = limitRate(kMaxJointVelocity[i], kMaxJointAcceleration[i], kMaxJointJerk[i],
                           commanded_positions[i], last_commanded_positions[i],
                           last_commanded_velocities[i], last_commanded_accelerations[i]);
  }
  return limited;
}

}  // namespace libfranka_080
}  // namespace franka
