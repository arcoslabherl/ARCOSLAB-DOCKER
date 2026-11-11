# Fake libfranka: find_package(Franka) config file.
include(CMakeFindDependencyMacro)
find_dependency(Threads)
include("${CMAKE_CURRENT_LIST_DIR}/FrankaTargets.cmake")

get_filename_component(_fake_franka_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(Franka_FOUND TRUE)
set(Franka_VERSION "0.0.0-fake")
set(Franka_INCLUDE_DIRS "${_fake_franka_prefix}/include")
set(Franka_LIBRARIES Franka::Franka)
message(STATUS "Using the FAKE libfranka from ${_fake_franka_prefix}")
