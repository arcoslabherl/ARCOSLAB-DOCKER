// Fake libfranka (test double). Same names as libfranka's exceptions.
#pragma once

#include <stdexcept>
#include <string>

namespace franka {

struct Exception : public std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct CommandException : public Exception {
  using Exception::Exception;
};
struct NetworkException : public Exception {
  using Exception::Exception;
};
struct ControlException : public Exception {
  using Exception::Exception;
};
struct RealtimeException : public Exception {
  using Exception::Exception;
};
struct InvalidOperationException : public Exception {
  using Exception::Exception;
};

}  // namespace franka
