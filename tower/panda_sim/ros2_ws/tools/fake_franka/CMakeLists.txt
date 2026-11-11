cmake_minimum_required(VERSION 3.8)
project(fake_franka CXX)

# Fake libfranka for testing panda_driver_topics without a robot.
# Installs a "Franka" CMake package (target Franka::Franka), so the driver
# builds against it unchanged. See README.md.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)

add_library(Franka SHARED src/robot.cpp src/gripper.cpp)
set_target_properties(Franka PROPERTIES OUTPUT_NAME franka)
target_include_directories(Franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(Franka PUBLIC Threads::Threads)

install(TARGETS Franka EXPORT FrankaTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT FrankaTargets NAMESPACE Franka:: DESTINATION lib/cmake/Franka)
install(FILES FrankaConfig.cmake DESTINATION lib/cmake/Franka)
