// Fake libfranka Robot: runs the driver's motion generator at 1 kHz and checks
// every command the way the Panda's controller would. See README.md.

#include <franka/exception.h>
#include <franka/robot.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace franka {

namespace {

constexpr int kJoints = 7;
constexpr double kDt = 0.001;  // the robot always assumes 1 ms per command

// Franka Panda (FER) limits, as in libfranka's rate_limiting.h for the Panda
// and Franka's datasheet. Verify against the docs of the libfranka version
// that will run on the tower.
constexpr std::array<double, kJoints> kMaxVelocity{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
constexpr std::array<double, kJoints> kMaxAcceleration{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
constexpr std::array<double, kJoints> kMaxJerk{7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0};
constexpr std::array<double, kJoints> kLowerLimit{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, kJoints> kUpperLimit{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

// Callback time above which the real robot starts losing packets.
constexpr double kCallbackBudgetUs = 300.0;

enum Violation { kVel = 0, kAcc, kJerk, kPos, kNumViolations };
const char* const kViolationNames[kNumViolations] = {
    "joint_motion_generator_velocity_limits_violation",
    "joint_motion_generator_velocity_discontinuity",      // acceleration over the limit
    "joint_motion_generator_acceleration_discontinuity",  // jerk over the limit
    "joint_position_limits_violation",
};

std::string env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr ? std::string(value) : std::string(fallback);
}

double env_double(const char* name, double fallback) {
  const char* value = std::getenv(name);
  return value != nullptr ? std::atof(value) : fallback;
}

std::array<double, kJoints> parse_pose(const std::string& text) {
  std::array<double, kJoints> pose{};
  std::istringstream stream(text);
  for (int i = 0; i < kJoints; i++) {
    if (!(stream >> pose[i])) {
      std::fprintf(stderr, "[fake_franka] bad FAKE_FRANKA_Q0 '%s'\n", text.c_str());
      std::exit(2);
    }
  }
  return pose;
}

std::string fmt7(const std::array<double, kJoints>& v, int precision = 4) {
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed;
  for (int i = 0; i < kJoints; i++) {
    out << (i ? " " : "") << v[i];
  }
  return out.str();
}

}  // namespace

struct Robot::Impl {
  std::mutex mutex;
  std::array<double, kJoints> q{};    // "measured" position
  std::array<double, kJoints> q_d{};  // last command accepted
  bool strict = false;
  bool in_control = false;
  double tau = 0.01;
  int report_max = 5;
  std::string reflex_file;
  std::string summary_file;
  FILE* log = nullptr;
  int segment = 0;
  bool error = false;  // a reflex happened; automaticErrorRecovery() clears it

  // Statistics of the current control() call
  struct Stats {
    uint64_t cycles = 0;
    uint64_t missed = 0;  // cycles lost because the loop overran (host jitter)
    uint64_t slow_callbacks = 0;
    double max_callback_us = 0.0;
    uint64_t stalls = 0;  // command identical to the previous one while moving
    std::array<double, kJoints> peak_v{}, peak_a{}, peak_j{};
    std::array<uint64_t, kNumViolations> violations{};
  } stats;

  void print_summary(const char* why) {
    const Stats& s = stats;
    std::fprintf(stderr,
                 "[fake_franka] ---- summary (segment %d, %s) ----\n"
                 "[fake_franka] cycles %llu (%.1f s), missed %llu, callbacks > %.0f us: %llu, "
                 "slowest %.0f us, stalls %llu\n",
                 segment, why, (unsigned long long)s.cycles, s.cycles * kDt,
                 (unsigned long long)s.missed, kCallbackBudgetUs,
                 (unsigned long long)s.slow_callbacks, s.max_callback_us,
                 (unsigned long long)s.stalls);
    std::array<double, kJoints> rv{}, ra{}, rj{};
    for (int i = 0; i < kJoints; i++) {
      rv[i] = s.peak_v[i] / kMaxVelocity[i];
      ra[i] = s.peak_a[i] / kMaxAcceleration[i];
      rj[i] = s.peak_j[i] / kMaxJerk[i];
    }
    std::fprintf(stderr, "[fake_franka] peak |v| rad/s   %s  (fraction of limit %s)\n",
                 fmt7(s.peak_v, 3).c_str(), fmt7(rv, 3).c_str());
    std::fprintf(stderr, "[fake_franka] peak |a| rad/s2  %s  (fraction of limit %s)\n",
                 fmt7(s.peak_a, 2).c_str(), fmt7(ra, 3).c_str());
    std::fprintf(stderr, "[fake_franka] peak |j| rad/s3  %s  (fraction of limit %s)\n",
                 fmt7(s.peak_j, 1).c_str(), fmt7(rj, 3).c_str());
    for (int k = 0; k < kNumViolations; k++) {
      std::fprintf(stderr, "[fake_franka] %-52s %llu\n", kViolationNames[k],
                   (unsigned long long)s.violations[k]);
    }
    if (log != nullptr) {
      std::fflush(log);
    }
  }
};

Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t) : impl_(new Impl) {
  if (env("FAKE_FRANKA_UNREACHABLE", "0") == "1") {
    throw NetworkException("libfranka (fake): Connection to FCI failed. Robot unreachable at " +
                           franka_address);
  }
  impl_->q = parse_pose(env("FAKE_FRANKA_Q0", "0 0 0 -1.57 0 1.57 0.71"));
  impl_->q_d = impl_->q;
  impl_->strict = env("FAKE_FRANKA_STRICT", "0") == "1";
  impl_->tau = env_double("FAKE_FRANKA_TAU", 0.01);
  impl_->report_max = static_cast<int>(env_double("FAKE_FRANKA_REPORT_MAX", 5));
  impl_->reflex_file = env("FAKE_FRANKA_REFLEX_FILE", "/tmp/fake_franka_reflex");
  impl_->summary_file = env("FAKE_FRANKA_SUMMARY_FILE", "/tmp/fake_franka_summary");
  std::string log_path = env("FAKE_FRANKA_LOG", "");
  if (!log_path.empty()) {
    impl_->log = std::fopen(log_path.c_str(), "w");
    if (impl_->log == nullptr) {
      std::fprintf(stderr, "[fake_franka] cannot open log %s\n", log_path.c_str());
      std::exit(2);
    }
    std::fprintf(impl_->log, "seg,k,t,step_ms,cb_us");
    for (int i = 0; i < kJoints; i++) std::fprintf(impl_->log, ",c%d", i);
    for (int i = 0; i < kJoints; i++) std::fprintf(impl_->log, ",q%d", i);
    std::fprintf(impl_->log, "\n");
  }
  std::fprintf(stderr,
               "[fake_franka] Robot '%s' (FAKE). Start pose %s, strict %d, tau %.3f s, "
               "reflex trigger file %s, log %s\n",
               franka_address.c_str(), fmt7(impl_->q).c_str(), impl_->strict, impl_->tau,
               impl_->reflex_file.c_str(), log_path.empty() ? "(none)" : log_path.c_str());
}

Robot::~Robot() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->log != nullptr) {
    std::fflush(impl_->log);
  }
}

RobotState Robot::readOnce() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->in_control) {
    throw InvalidOperationException("libfranka (fake): readOnce while a control loop runs");
  }
  RobotState state;
  state.q = impl_->q;
  state.q_d = impl_->q_d;
  return state;
}

void Robot::automaticErrorRecovery() {
  std::fprintf(stderr, "[fake_franka] automaticErrorRecovery\n");
  impl_->error = false;
}

void Robot::setCollisionBehavior(const std::array<double, 7>&, const std::array<double, 7>&,
                                 const std::array<double, 6>&, const std::array<double, 6>&) {
  std::fprintf(stderr, "[fake_franka] setCollisionBehavior (4 args)\n");
}

void Robot::setCollisionBehavior(const std::array<double, 7>&, const std::array<double, 7>&,
                                 const std::array<double, 7>&, const std::array<double, 7>&,
                                 const std::array<double, 6>&, const std::array<double, 6>&,
                                 const std::array<double, 6>&, const std::array<double, 6>&) {
  std::fprintf(stderr, "[fake_franka] setCollisionBehavior (8 args)\n");
}

void Robot::setJointImpedance(const std::array<double, 7>& K) {
  std::fprintf(stderr, "[fake_franka] setJointImpedance %s\n", fmt7(K, 0).c_str());
}

void Robot::setCartesianImpedance(const std::array<double, 6>&) {
  std::fprintf(stderr, "[fake_franka] setCartesianImpedance\n");
}

void Robot::control(JointPositionsCallback callback, ControllerMode, bool limit_rate,
                    double cutoff_frequency) {
  Impl& m = *impl_;
  if (m.error) {
    throw CommandException(
        "libfranka (fake): Move command rejected: command not possible in the current mode "
        "(reflex). Call automaticErrorRecovery() first.");
  }
  {
    std::lock_guard<std::mutex> lock(m.mutex);
    m.in_control = true;
    m.segment++;
    m.stats = Impl::Stats{};
  }
  std::fprintf(stderr, "[fake_franka] control() starts, segment %d (limit_rate %d, cutoff %.0f)\n",
               m.segment, limit_rate, cutoff_frequency);
  if (limit_rate) {
    std::fprintf(stderr,
                 "[fake_franka] note: the real libfranka would rate-limit (clip) the commands; "
                 "this fake does not, it reports the raw stream\n");
  }

  RobotState state;
  state.q = m.q;
  state.q_d = m.q_d;
  std::array<double, kJoints> prev = m.q_d, prev_v{}, prev_a{};
  std::array<int, kNumViolations> reported{};
  uint64_t step_ms = 0;  // libfranka's first call gets a zero duration
  uint64_t k = 0;
  double t = 0.0;
  auto next = std::chrono::steady_clock::now();

  auto finish = [&](const char* why) {
    std::lock_guard<std::mutex> lock(m.mutex);
    m.in_control = false;
    m.print_summary(why);
  };

  try {
    while (true) {
      // Trigger files, checked every 10 cycles (cheap enough at 1 kHz)
      FILE* trigger = k % 10 == 0 ? std::fopen(m.summary_file.c_str(), "r") : nullptr;
      if (trigger != nullptr) {
        std::fclose(trigger);
        std::remove(m.summary_file.c_str());
        std::lock_guard<std::mutex> lock(m.mutex);
        m.print_summary("requested");
      }
      trigger = k % 10 == 0 ? std::fopen(m.reflex_file.c_str(), "r") : nullptr;
      if (trigger != nullptr) {
        std::fclose(trigger);
        std::remove(m.reflex_file.c_str());
        m.error = true;
        throw ControlException(
            "libfranka (fake): Move command aborted: motion aborted by reflex! "
            "[\"cartesian_reflex\"] (injected)");
      }

      t += step_ms * kDt;
      state.time = Duration(static_cast<uint64_t>(std::llround(t * 1000.0)));
      auto t0 = std::chrono::steady_clock::now();
      JointPositions command = callback(state, Duration(step_ms));
      double cb_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();

      Impl::Stats& s = m.stats;
      s.cycles++;
      s.max_callback_us = std::max(s.max_callback_us, cb_us);
      if (cb_us > kCallbackBudgetUs) {
        s.slow_callbacks++;
      }

      std::array<double, kJoints> v{}, a{}, j{};
      bool moving_before = false, same = true;
      for (int i = 0; i < kJoints; i++) {
        double c = command.q[i];
        if (!std::isfinite(c)) {
          m.error = true;
          throw ControlException("libfranka (fake): command is not finite (NaN/inf)");
        }
        v[i] = (c - prev[i]) / kDt;
        a[i] = (v[i] - prev_v[i]) / kDt;
        j[i] = (a[i] - prev_a[i]) / kDt;
        moving_before = moving_before || std::fabs(prev_v[i]) > 1e-3;
        same = same && c == prev[i];
        s.peak_v[i] = std::max(s.peak_v[i], std::fabs(v[i]));
        s.peak_a[i] = std::max(s.peak_a[i], std::fabs(a[i]));
        s.peak_j[i] = std::max(s.peak_j[i], std::fabs(j[i]));
        const bool bad[kNumViolations] = {
            std::fabs(v[i]) > kMaxVelocity[i],
            std::fabs(a[i]) > kMaxAcceleration[i],
            std::fabs(j[i]) > kMaxJerk[i],
            c < kLowerLimit[i] || c > kUpperLimit[i],
        };
        for (int type = 0; type < kNumViolations; type++) {
          if (!bad[type]) continue;
          s.violations[type]++;
          if (reported[type] < m.report_max) {
            reported[type]++;
            std::fprintf(stderr,
                         "[fake_franka] VIOLATION %s: t=%.3f s J%d cmd %.5f v %.3f a %.2f j %.1f%s\n",
                         kViolationNames[type], t, i + 1, c, v[i], a[i], j[i],
                         reported[type] == m.report_max ? " (further ones only counted)" : "");
          }
          if (m.strict) {
            m.error = true;
            throw ControlException(std::string("libfranka (fake): Move command aborted: motion "
                                               "aborted by reflex! [\"") +
                                   kViolationNames[type] + "\"]");
          }
        }
      }
      if (same && moving_before) {
        s.stalls++;
        if (s.stalls <= static_cast<uint64_t>(m.report_max)) {
          std::fprintf(stderr,
                       "[fake_franka] STALL: t=%.3f s, command repeated while moving "
                       "(|v| was up to %.3f rad/s)\n",
                       t, *std::max_element(prev_v.begin(), prev_v.end(),
                                            [](double x, double y) { return std::fabs(x) < std::fabs(y); }));
        }
      }

      // The robot follows the command with a first-order lag.
      {
        std::lock_guard<std::mutex> lock(m.mutex);
        for (int i = 0; i < kJoints; i++) {
          m.q_d[i] = command.q[i];
          m.q[i] += (m.q_d[i] - m.q[i]) * std::min(1.0, kDt / m.tau);
        }
        state.q = m.q;
        state.q_d = m.q_d;
        state.dq_d = v;
        state.ddq_d = a;
        state.dq = v;
      }
      prev = command.q;
      prev_v = v;
      prev_a = a;

      if (m.log != nullptr) {
        std::fprintf(m.log, "%d,%llu,%.3f,%llu,%.1f", m.segment, (unsigned long long)k, t,
                     (unsigned long long)step_ms, cb_us);
        for (int i = 0; i < kJoints; i++) std::fprintf(m.log, ",%.7f", command.q[i]);
        for (int i = 0; i < kJoints; i++) std::fprintf(m.log, ",%.7f", state.q[i]);
        std::fprintf(m.log, "\n");
        if (k % 1000 == 0) std::fflush(m.log);
      }
      k++;

      if (command.motion_finished) {
        finish("motion finished");
        return;
      }

      // 1 kHz. If the loop overran, the lost cycles count as lost packets.
      next += std::chrono::milliseconds(1);
      auto now = std::chrono::steady_clock::now();
      step_ms = 1;
      if (now > next + std::chrono::milliseconds(1)) {
        uint64_t lost = static_cast<uint64_t>((now - next) / std::chrono::milliseconds(1));
        s.missed += lost;
        step_ms += lost;
        next += std::chrono::milliseconds(lost);
      }
      std::this_thread::sleep_until(next);
    }
  } catch (const Exception& ex) {
    std::fprintf(stderr, "[fake_franka] control() throws: %s\n", ex.what());
    finish("exception");
    throw;
  }
}

}  // namespace franka
