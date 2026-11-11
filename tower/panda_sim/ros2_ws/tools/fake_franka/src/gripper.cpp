// Fake libfranka Gripper (Franka Hand). Blocking calls, like libfranka.
// See README.md for the environment variables.

#include <franka/exception.h>
#include <franka/gripper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

namespace franka {

namespace {

constexpr double kMaxWidth = 0.08;  // m
constexpr double kStep = 0.01;      // s, simulation step of a move

double env_double(const char* name, double fallback) {
  const char* value = std::getenv(name);
  return value != nullptr ? std::atof(value) : fallback;
}

}  // namespace

struct Gripper::Impl {
  mutable std::mutex mutex;
  double width = 0.0;
  bool is_grasped = false;
  double object = 0.0;       // m, width of an object between the fingers (0 = none)
  double max_speed = 0.1;    // m/s
  double homing_time = 2.0;  // s
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  // Moves towards target at speed. Returns the final width. Stops at the
  // object when closing onto it.
  double run(double target, double speed, bool* blocked) {
    speed = std::min(std::fabs(speed), max_speed);
    *blocked = false;
    while (true) {
      double current;
      {
        std::lock_guard<std::mutex> lock(mutex);
        current = width;
      }
      double stop_at = target;
      if (object > 0.0 && current >= object - 1e-9 && target < object) {
        stop_at = object;
      }
      double error = stop_at - current;
      if (std::fabs(error) < 1e-6 || speed <= 0.0) {
        *blocked = stop_at != target;
        return current;
      }
      double delta = std::copysign(std::min(std::fabs(error), speed * kStep), error);
      {
        std::lock_guard<std::mutex> lock(mutex);
        width = current + delta;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(kStep));
    }
  }
};

Gripper::Gripper(const std::string& franka_address) : impl_(new Impl) {
  impl_->width = env_double("FAKE_GRIPPER_WIDTH0", 0.04);
  impl_->object = env_double("FAKE_GRIPPER_OBJECT", 0.0);
  impl_->max_speed = env_double("FAKE_GRIPPER_MAX_SPEED", 0.1);
  impl_->homing_time = env_double("FAKE_GRIPPER_HOMING_TIME", 2.0);
  std::fprintf(stderr, "[fake_franka] Gripper '%s' (FAKE). Width %.3f m, object %.3f m\n",
               franka_address.c_str(), impl_->width, impl_->object);
}

Gripper::~Gripper() = default;

bool Gripper::homing() const {
  std::fprintf(stderr,
               "[fake_franka] GRIPPER HOMING: the real gripper closes and opens fully here "
               "(%.1f s)\n",
               impl_->homing_time);
  std::this_thread::sleep_for(std::chrono::duration<double>(impl_->homing_time));
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->width = kMaxWidth;
  impl_->is_grasped = false;
  return true;
}

bool Gripper::move(double width, double speed) const {
  std::fprintf(stderr, "[fake_franka] gripper move to %.4f m at %.3f m/s\n", width, speed);
  if (width < 0.0 || width > kMaxWidth || speed <= 0.0) {
    throw CommandException("libfranka (fake): gripper move: invalid width or speed");
  }
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->is_grasped = false;
  }
  bool blocked = false;
  double final_width = impl_->run(width, speed, &blocked);
  if (blocked) {
    std::fprintf(stderr, "[fake_franka] gripper move FAILED: blocked by the object at %.4f m\n",
                 final_width);
    return false;
  }
  return true;
}

bool Gripper::grasp(double width, double speed, double force, double epsilon_inner,
                    double epsilon_outer) const {
  std::fprintf(stderr,
               "[fake_franka] gripper grasp width %.4f m, speed %.3f m/s, force %.1f N, "
               "eps %.4f/%.4f\n",
               width, speed, force, epsilon_inner, epsilon_outer);
  if (width < 0.0 || width > kMaxWidth || speed <= 0.0 || force <= 0.0) {
    throw CommandException("libfranka (fake): gripper grasp: invalid arguments");
  }
  bool blocked = false;
  double final_width = impl_->run(width, speed, &blocked);
  bool ok = final_width > width - epsilon_inner && final_width < width + epsilon_outer;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->is_grasped = ok;
  }
  std::fprintf(stderr, "[fake_franka] gripper grasp %s at %.4f m\n", ok ? "holds" : "FAILED",
               final_width);
  return ok;
}

bool Gripper::stop() const {
  std::fprintf(stderr, "[fake_franka] gripper stop\n");
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->is_grasped = false;
  return true;
}

GripperState Gripper::readOnce() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  GripperState state;
  state.width = impl_->width;
  state.max_width = kMaxWidth;
  state.is_grasped = impl_->is_grasped;
  state.temperature = 30;
  state.time = Duration(static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                            impl_->start)
          .count()));
  return state;
}

}  // namespace franka
