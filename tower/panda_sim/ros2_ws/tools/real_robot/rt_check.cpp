// Runs the two real-time checks libfranka does before its control loop when
// the robot is created with the default RealtimeConfig::kEnforce (as the lab
// driver does): this thread at the highest SCHED_FIFO priority, and a
// real-time kernel (/sys/kernel/realtime). No robot needed.
#include <franka/control_tools.h>

#include <iostream>
#include <string>

int main() {
  std::string error;
  const bool priority = franka::setCurrentThreadToHighestSchedulerPriority(&error);
  const bool kernel = franka::hasRealtimeKernel();

  std::cout << "Highest SCHED_FIFO priority: "
            << (priority ? "OK" : "FAILED (" + error + ")") << "\n"
            << "Real-time kernel (/sys/kernel/realtime): "
            << (kernel ? "OK" : "NO") << "\n";

  if (priority && kernel) {
    std::cout << "libfranka's real-time checks pass.\n";
    return 0;
  }
  std::cout << "libfranka would refuse to control the robot.\n";
  return 1;
}
