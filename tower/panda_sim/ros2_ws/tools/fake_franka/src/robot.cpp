// Fake libfranka Robot: runs the driver's motion generator at 1 kHz and checks
// every command the way the Panda's controller would. See README.md.

#include <franka/exception.h>
#include <franka/robot.h>

#include "rate_limiting.h"

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
  int lose_every = 0;  // drop 2 packets every N cycles (FAKE_FRANKA_LOSE_EVERY), 0 = never
  bool extrapolate_acceleration = true;  // FAKE_FRANKA_EXTRAPOLATE: acceleration | velocity
  int limit_rate_override = -1;          // FAKE_FRANKA_LIMIT_RATE: unset = as the caller asks
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
    uint64_t limited = 0;  // cycles where the rate limiter changed the command
    double max_limited = 0.0;  // largest change it made (rad)
    uint64_t saturated = 0;  // cycles with |ddq_d| at >= 90 % of the limit
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
    std::fprintf(stderr,
                 "[fake_franka] rate limiter changed %llu commands (largest %.6f rad), "
                 "|ddq_d| >= 90%% of the limit in %llu cycles\n",
                 (unsigned long long)s.limited, s.max_limited, (unsigned long long)s.saturated);
    std::array<double, kJoints> rv{}, ra{}, rj{};
    for (int i = 0; i < kJoints; i++) {
      rv[i] = s.peak_v[i] / kMaxVelocity[i];
      ra[i] = s.peak_a[i] / kMaxAcceleration[i];
      rj[i] = s.peak_j[i] / kMaxJerk[i];
    }
    std::fprintf(stderr, "[fake_franka] raw peak |v| rad/s   %s  (fraction of limit %s)\n",
                 fmt7(s.peak_v, 3).c_str(), fmt7(rv, 3).c_str());
    std::fprintf(stderr, "[fake_franka] raw peak |a| rad/s2  %s  (fraction of limit %s)\n",
                 fmt7(s.peak_a, 2).c_str(), fmt7(ra, 3).c_str());
    std::fprintf(stderr, "[fake_franka] raw peak |j| rad/s3  %s  (fraction of limit %s)\n",
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
  impl_->lose_every = static_cast<int>(env_double("FAKE_FRANKA_LOSE_EVERY", 0));
  const std::string extrapolate = env("FAKE_FRANKA_EXTRAPOLATE", "acceleration");
  if (extrapolate != "acceleration" && extrapolate != "velocity") {
    std::fprintf(stderr, "[fake_franka] bad FAKE_FRANKA_EXTRAPOLATE '%s'\n", extrapolate.c_str());
    std::exit(2);
  }
  impl_->extrapolate_acceleration = extrapolate == "acceleration";
  const std::string limit_rate = env("FAKE_FRANKA_LIMIT_RATE", "");
  if (!limit_rate.empty()) {
    impl_->limit_rate_override = limit_rate == "0" ? 0 : 1;
  }
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
    for (int i = 0; i < kJoints; i++) std::fprintf(impl_->log, ",d%d", i);
    std::fprintf(impl_->log, "\n");
  }
  std::fprintf(stderr,
               "[fake_franka] Robot '%s' (FAKE). Start pose %s, strict %d, tau %.3f s, "
               "lost packets extrapolate %s, reflex trigger file %s, log %s\n",
               franka_address.c_str(), fmt7(impl_->q).c_str(), impl_->strict, impl_->tau,
               extrapolate.c_str(),
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
  const bool limit = m.limit_rate_override >= 0 ? m.limit_rate_override == 1 : limit_rate;
  std::fprintf(stderr,
               "[fake_franka] control() starts, segment %d (limit_rate %d%s, cutoff %.0f%s)\n",
               m.segment, limit, m.limit_rate_override >= 0 ? " forced by FAKE_FRANKA_LIMIT_RATE" : "",
               cutoff_frequency, cutoff_frequency < 1000.0 ? ": the real low-pass filter is NOT modelled" : ", filter off");

  // The robot's side: the last command it accepted and its derivatives. The
  // real libfranka sends these back as q_d, dq_d, ddq_d, and its rate limiter
  // works from them.
  std::array<double, kJoints> q_d = m.q_d, dq_d{}, ddq_d{};
  RobotState state;
  state.q = m.q;
  state.q_d = q_d;
  std::array<double, kJoints> prev_raw = q_d;
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

      const std::array<double, kJoints> raw = command.q;
      for (int i = 0; i < kJoints; i++) {
        if (!std::isfinite(raw[i])) {
          m.error = true;
          throw ControlException("libfranka (fake): command is not finite (NaN/inf)");
        }
      }
      // What the robot receives: the command after libfranka's rate limiter
      const std::array<double, kJoints> c =
          limit ? libfranka_080::limitRate(raw, q_d, dq_d, ddq_d) : raw;

      // Violations and peaks are of the caller's raw command, judged from the
      // robot's q_d/dq_d/ddq_d: a violation is a caller (driver) problem that
      // the rate limiter, when on, has to fix. v and a are what the robot
      // accepted (after the limiter).
      std::array<double, kJoints> v{}, a{};
      bool moving_before = false, same = true, changed = false, saturated = false;
      for (int i = 0; i < kJoints; i++) {
        const double rv = (raw[i] - q_d[i]) / kDt;
        const double ra = (rv - dq_d[i]) / kDt;
        const double rj = (ra - ddq_d[i]) / kDt;
        v[i] = (c[i] - q_d[i]) / kDt;
        a[i] = (v[i] - dq_d[i]) / kDt;
        moving_before = moving_before || std::fabs(dq_d[i]) > 1e-3;
        same = same && raw[i] == prev_raw[i];
        const double change = std::fabs(c[i] - raw[i]);
        if (change > 1e-12) {
          changed = true;
          s.max_limited = std::max(s.max_limited, change);
        }
        saturated = saturated || std::fabs(a[i]) >= 0.9 * kMaxAcceleration[i];
        s.peak_v[i] = std::max(s.peak_v[i], std::fabs(rv));
        s.peak_a[i] = std::max(s.peak_a[i], std::fabs(ra));
        s.peak_j[i] = std::max(s.peak_j[i], std::fabs(rj));
        const bool bad[kNumViolations] = {
            std::fabs(rv) > kMaxVelocity[i],
            std::fabs(ra) > kMaxAcceleration[i],
            std::fabs(rj) > kMaxJerk[i],
            raw[i] < kLowerLimit[i] || raw[i] > kUpperLimit[i],
        };
        for (int type = 0; type < kNumViolations; type++) {
          if (!bad[type]) continue;
          s.violations[type]++;
          if (reported[type] < m.report_max) {
            reported[type]++;
            std::fprintf(stderr,
                         "[fake_franka] VIOLATION %s: t=%.3f s J%d cmd %.5f v %.3f a %.2f j %.1f%s\n",
                         kViolationNames[type], t, i + 1, raw[i], rv, ra, rj,
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
      s.limited += changed ? 1 : 0;
      s.saturated += saturated ? 1 : 0;
      if (same && moving_before) {
        s.stalls++;
        if (s.stalls <= static_cast<uint64_t>(m.report_max)) {
          std::fprintf(stderr,
                       "[fake_franka] STALL: t=%.3f s, command repeated while moving "
                       "(|dq_d| was up to %.3f rad/s)\n",
                       t, *std::max_element(dq_d.begin(), dq_d.end(),
                                            [](double x, double y) { return std::fabs(x) < std::fabs(y); }));
        }
      }
      prev_raw = raw;
      q_d = c;
      dq_d = v;
      ddq_d = a;

      // The robot follows q_d with a first-order lag.
      {
        std::lock_guard<std::mutex> lock(m.mutex);
        for (int i = 0; i < kJoints; i++) {
          m.q_d[i] = q_d[i];
          m.q[i] += (m.q_d[i] - m.q[i]) * std::min(1.0, kDt / m.tau);
        }
        state.q = m.q;
        state.q_d = q_d;
        state.dq_d = dq_d;
        state.ddq_d = ddq_d;
        state.dq = dq_d;
      }

      if (m.log != nullptr) {
        std::fprintf(m.log, "%d,%llu,%.3f,%llu,%.1f", m.segment, (unsigned long long)k, t,
                     (unsigned long long)step_ms, cb_us);
        for (int i = 0; i < kJoints; i++) std::fprintf(m.log, ",%.7f", raw[i]);
        for (int i = 0; i < kJoints; i++) std::fprintf(m.log, ",%.7f", state.q[i]);
        for (int i = 0; i < kJoints; i++) std::fprintf(m.log, ",%.7f", q_d[i]);
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
      uint64_t lost = 0;
      if (now > next + std::chrono::milliseconds(1)) {
        lost = static_cast<uint64_t>((now - next) / std::chrono::milliseconds(1));
      }
      if (m.lose_every > 0 && k % static_cast<uint64_t>(m.lose_every) == 0) {
        lost += 2;  // forced, to test lost packets on any computer
      }
      if (lost > 0) {
        s.missed += lost;
        step_ms += lost;
        next += std::chrono::milliseconds(lost);
        // While packets are lost the robot goes on by itself, so q_d moves
        // ahead and the next command is judged (and rate-limited) from there.
        // libfranka 0.8.0's rate_limiting.h: "When a packet is lost, FCI
        // assumes a constant acceleration model"; FAKE_FRANKA_EXTRAPOLATE=
        // velocity keeps the velocity instead (our first model, 2026-11-17).
        std::lock_guard<std::mutex> lock(m.mutex);
        for (uint64_t n = 0; n < lost; n++) {
          for (int i = 0; i < kJoints; i++) {
            if (m.extrapolate_acceleration) {
              dq_d[i] += ddq_d[i] * kDt;
            }
            q_d[i] += dq_d[i] * kDt;
            m.q[i] += (q_d[i] - m.q[i]) * std::min(1.0, kDt / m.tau);
          }
        }
        m.q_d = q_d;
        state.q = m.q;
        state.q_d = q_d;
        state.dq_d = dq_d;
        state.dq = dq_d;
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
