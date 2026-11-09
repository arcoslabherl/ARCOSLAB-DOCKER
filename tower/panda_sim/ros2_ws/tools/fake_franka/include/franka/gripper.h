// Fake libfranka (test double): the Franka Hand. See tools/fake_franka/README.md.
#pragma once

#include <string>

#include <franka/robot.h>

namespace franka {

struct GripperState {
  double width{0.0};
  double max_width{0.0};
  bool is_grasped{false};
  uint16_t temperature{0};
  Duration time{};
};

class Gripper {
 public:
  explicit Gripper(const std::string& franka_address);
  ~Gripper();

  bool homing() const;
  bool move(double width, double speed) const;
  bool grasp(double width, double speed, double force,
             double epsilon_inner = 0.005, double epsilon_outer = 0.005) const;
  bool stop() const;
  GripperState readOnce() const;

  struct Impl;

 private:
  Impl* impl_;  // leaked on purpose, see Robot
};

}  // namespace franka
