// Moves one joint of the Panda a few degrees from where it is, and back, with
// libfranka alone (no driver, no ROS). The first, smallest motion test.
//
//   nudge <robot ip> <joint 1-7> <degrees> [--yes]
//
// Same robot settings as the lab driver (collision thresholds, joint
// impedance, joint position control with libfranka's rate limiter), so it
// shows how the arm behaves under the driver. Out in 3 s, 1 s still, back in
// 3 s (counted in 1 ms control cycles), each half a minimum-jerk curve (zero
// velocity and acceleration at both ends). At most 10 degrees, and only inside
// the joint limits minus 0.1 rad.
//
// At the end it prints the communication quality, as libfranka's
// communication_test does: control_command_success_rate and lost packets.
#include <franka/exception.h>
#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

namespace {

constexpr double kMaxDegrees = 10.0;
constexpr double kMargin = 0.1;      // rad, as the calibrator's joint_limit_margin
constexpr double kMoveTime = 3.0;    // s, each way
constexpr double kHoldTime = 1.0;    // s, at the target
constexpr double kStill = 0.01;      // rad/s, largest joint speed to start

// Franka Panda joint position limits (rad), as in the URDF and the calibrator
constexpr std::array<double, 7> kLow{{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
constexpr std::array<double, 7> kHigh{{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};

double deg(double rad) { return rad * 180.0 / M_PI; }

// Minimum-jerk position from 0 to 1, s in [0, 1]
double min_jerk(double s) {
  s = std::clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// The lab driver's setDefaultBehavior() (ros2_panda_driver_topics.cpp): its
// second setCollisionBehavior() replaces the first one.
void driver_behavior(franka::Robot& robot) {
  robot.setJointImpedance({{500, 500, 500, 500, 500, 200, 200}});
  robot.setCartesianImpedance({{500, 500, 500, 50, 50, 50}});
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 30.0, 20.0}},
      {{40.0, 40.0, 48.0, 48.0, 46.0, 30.0, 30.0}}, {{80.0, 80.0, 100.0, 100.0, 100.0, 60.0, 60.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
      {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}}, {{80.0, 80.0, 80.0, 90.0, 90.0, 90.0}});
}

std::string pose_deg(const std::array<double, 7>& q) {
  std::string text;
  char buffer[16];
  for (double value : q) {
    std::snprintf(buffer, sizeof(buffer), "%7.1f", deg(value));
    text += buffer;
  }
  return text;
}

int usage() {
  std::cerr << "Usage: nudge <robot ip> <joint 1-7> <degrees> [--yes]\n"
            << "  Moves the joint by <degrees> (at most " << kMaxDegrees
            << ") and back.\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) return usage();
  const std::string ip = argv[1];
  int joint = 0;
  double degrees = 0.0;
  try {
    joint = std::stoi(argv[2]);
    degrees = std::stod(argv[3]);
  } catch (const std::exception&) {
    return usage();
  }
  const bool assume_yes = argc == 5 && std::string(argv[4]) == "--yes";
  if (argc == 5 && !assume_yes) return usage();

  if (joint < 1 || joint > 7) {
    std::cout << "NOT DONE: the joint must be 1 to 7.\n";
    return 1;
  }
  if (degrees == 0.0 || std::abs(degrees) > kMaxDegrees) {
    std::cout << "NOT DONE: the angle must be between -" << kMaxDegrees << " and "
              << kMaxDegrees << " degrees, not 0.\n";
    return 1;
  }
  const int j = joint - 1;
  const double delta = degrees * M_PI / 180.0;

  try {
    franka::Robot robot(ip);
    const franka::RobotState start = robot.readOnce();

    const double target = start.q[j] + delta;
    if (target < kLow[j] + kMargin || target > kHigh[j] - kMargin) {
      std::printf("NOT DONE: J%d would end at %.1f deg, past its limit minus %.1f deg "
                  "(%.1f to %.1f).\n",
                  joint, deg(target), deg(kMargin), deg(kLow[j] + kMargin),
                  deg(kHigh[j] - kMargin));
      return 1;
    }
    double speed = 0.0;
    for (double v : start.dq) speed = std::max(speed, std::abs(v));
    if (speed > kStill) {
      std::printf("NOT DONE: the arm moves (%.3f rad/s). Wait until it's still.\n", speed);
      return 1;
    }

    std::printf("Pose now (deg):  %s\n", pose_deg(start.q).c_str());
    std::printf("J%d: %.1f -> %.1f deg and back (%.0f s each way, %.0f s at the target).\n",
                joint, deg(start.q[j]), deg(target), kMoveTime, kHoldTime);
    std::printf("Peak speed %.3f rad/s. Nothing else moves. E-stop in hand.\n",
                1.875 * std::abs(delta) / kMoveTime);
    if (!assume_yes) {
      std::cout << "Type 'yes' to start: " << std::flush;
      std::string answer;
      if (!std::getline(std::cin, answer) || answer != "yes") {
        std::cout << "NOT DONE: cancelled.\n";
        return 1;
      }
    }

    // As the driver does before each control loop
    robot.automaticErrorRecovery();
    driver_behavior(robot);

    const double total = 2.0 * kMoveTime + kHoldTime;
    std::array<double, 7> base{};
    double time = 0.0;
    double max_follow = 0.0;  // largest |q - q_d| (rad)
    double min_rate = 1.0, sum_rate = 0.0;
    uint64_t samples = 0, lost = 0;
    std::array<double, 7> last_q{};

    robot.control(
        [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
          if (period.toMSec() == 0) {
            base = state.q_d;  // start exactly where the robot's command is
          } else {
            // One step per cycle, as the driver's ruckig: after a lost
            // packet the curve goes on from the last command instead of
            // jumping ahead (a jump is a velocity/acceleration discontinuity)
            time += 0.001;
            lost += period.toMSec() - 1;
          }

          if (time > 0.1) {  // skip the first samples, like communication_test
            min_rate = std::min(min_rate, state.control_command_success_rate);
            sum_rate += state.control_command_success_rate;
            samples++;
          }
          for (int i = 0; i < 7; i++) {
            max_follow = std::max(max_follow, std::abs(state.q[i] - state.q_d[i]));
          }
          last_q = state.q;

          double s = 0.0;
          if (time < kMoveTime) {
            s = min_jerk(time / kMoveTime);
          } else if (time < kMoveTime + kHoldTime) {
            s = 1.0;
          } else {
            s = 1.0 - min_jerk((time - kMoveTime - kHoldTime) / kMoveTime);
          }
          std::array<double, 7> command = base;
          command[j] += delta * s;
          if (time >= total) return franka::MotionFinished(franka::JointPositions(base));
          return command;
        },
        franka::ControllerMode::kJointImpedance, true, 1000.0);

    double back = 0.0;
    for (int i = 0; i < 7; i++) back = std::max(back, std::abs(last_q[i] - base[i]));
    std::printf("Done. Back at the start within %.2f deg (largest joint).\n", deg(back));
    std::printf("Largest gap between the measured and commanded position: %.2f deg.\n",
                deg(max_follow));
    if (samples > 0) {
      std::printf("control_command_success_rate: average %.4f, minimum %.4f. Lost packets: %llu.\n",
                  sum_rate / samples, min_rate, static_cast<unsigned long long>(lost));
    }
    return 0;
  } catch (const franka::Exception& ex) {
    std::cout << "libfranka error: " << ex.what() << "\n"
              << "The robot stopped (reflex or no connection). Check the arm and the "
                 "area. The next run clears the error first, as the driver does.\n";
    return 1;
  }
}
