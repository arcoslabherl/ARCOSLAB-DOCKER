// Fake libfranka (test double) for running panda_driver_topics without a
// robot. Only the part of libfranka's API the driver uses, with the same
// names and signatures. See tools/fake_franka/README.md.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>

namespace franka {

class Duration {
 public:
  Duration() noexcept = default;
  explicit Duration(uint64_t milliseconds) noexcept : ms_(milliseconds) {}
  double toSec() const noexcept { return static_cast<double>(ms_) / 1000.0; }
  uint64_t toMSec() const noexcept { return ms_; }

 private:
  uint64_t ms_{0};
};

struct RobotState {
  std::array<double, 7> q{};
  std::array<double, 7> q_d{};
  std::array<double, 7> dq{};
  std::array<double, 7> dq_d{};
  std::array<double, 7> ddq_d{};
  std::array<double, 7> tau_J{};
  std::array<double, 7> tau_ext_hat_filtered{};
  Duration time{};
};

enum class ControllerMode { kJointImpedance, kCartesianImpedance };
enum class RealtimeConfig { kEnforce, kIgnore };

struct Finishable {
  bool motion_finished = false;
};

struct JointPositions : public Finishable {
  JointPositions(const std::array<double, 7>& joint_positions) noexcept  // NOLINT: implicit, like libfranka
      : q(joint_positions) {}
  JointPositions(std::initializer_list<double> joint_positions) {
    if (joint_positions.size() != q.size()) {
      throw std::invalid_argument("Invalid number of elements in joint_positions.");
    }
    std::size_t i = 0;
    for (double value : joint_positions) {
      q[i++] = value;
    }
  }
  std::array<double, 7> q{};
};

template <class T>
inline T MotionFinished(T command) noexcept {
  command.motion_finished = true;
  return command;
}

class Robot {
 public:
  using JointPositionsCallback =
      std::function<JointPositions(const RobotState&, franka::Duration)>;

  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce,
                 size_t log_size = 50);
  ~Robot();

  void control(JointPositionsCallback motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance,
               bool limit_rate = true,
               double cutoff_frequency = 100.0);

  RobotState readOnce();
  void automaticErrorRecovery();

  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds);
  void setCollisionBehavior(
      const std::array<double, 7>& lower_torque_thresholds_acceleration,
      const std::array<double, 7>& upper_torque_thresholds_acceleration,
      const std::array<double, 7>& lower_torque_thresholds_nominal,
      const std::array<double, 7>& upper_torque_thresholds_nominal,
      const std::array<double, 6>& lower_force_thresholds_acceleration,
      const std::array<double, 6>& upper_force_thresholds_acceleration,
      const std::array<double, 6>& lower_force_thresholds_nominal,
      const std::array<double, 6>& upper_force_thresholds_nominal);
  void setJointImpedance(const std::array<double, 7>& K_theta);
  void setCartesianImpedance(const std::array<double, 6>& K_x);

  struct Impl;

 private:
  Impl* impl_;  // deliberately leaked: the driver exit()s from a signal
                // handler while its control thread may still use it
};

}  // namespace franka
