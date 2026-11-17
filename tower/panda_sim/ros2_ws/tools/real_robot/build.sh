#!/usr/bin/env bash
# Builds the real driver (panda_driver_topics) against the image's libfranka,
# rt_check and nudge, in /ros2_ws/build/_real_robot. Doesn't touch the colcon
# workspace or the driver's COLCON_IGNORE. Run inside the real image's
# container ('make .build-real', 'make run REAL=1').
#   build.sh [<driver patch>]   also builds driver_patched/ with the patch
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

if [ ! -f /usr/local/lib/cmake/Franka/FrankaConfig.cmake ]; then
    echo "No libfranka in this container. Use the real image:"
    echo "    make stop && make .build-real && make run REAL=1"
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=/ros2_ws/build/_real_robot
DRIVER_SRC=/ros2_ws/src/panda-driver-topics/ros2_panda_driver_topics

mkdir -p "$OUT"
g++ -O2 -std=c++17 "$HERE/rt_check.cpp" -lfranka -o "$OUT/rt_check"
echo "Built: $OUT/rt_check"
g++ -O2 -std=c++17 "$HERE/nudge.cpp" -lfranka -o "$OUT/nudge"
echo "Built: $OUT/nudge"

cmake -S "$DRIVER_SRC" -B "$OUT/driver" -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/driver" -j"$(nproc)"
echo "Built: $OUT/driver/panda_driver_topics"

# Optionally, the same driver with a patch, built from a copy (src/ stays at
# the pinned commit). patches/ isn't mounted in the container: docker cp it.
PATCH="$1"
if [ -n "$PATCH" ]; then
    rm -rf "$OUT/src_patched"
    mkdir -p "$OUT/src_patched"
    cp -r "$DRIVER_SRC" "$OUT/src_patched/"
    rm -f "$OUT/src_patched/ros2_panda_driver_topics/COLCON_IGNORE"
    git -C "$OUT/src_patched" apply "$PATCH"
    cmake -S "$OUT/src_patched/ros2_panda_driver_topics" -B "$OUT/driver_patched" \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "$OUT/driver_patched" -j"$(nproc)"
    echo "Built: $OUT/driver_patched/panda_driver_topics (with $(basename "$PATCH"))"
fi
