#!/usr/bin/env bash
# Builds the fake libfranka and the real driver (panda_driver_topics) against
# it, in /ros2_ws/build/_fake_franka. Doesn't touch the colcon workspace or the
# driver's COLCON_IGNORE. Run inside the container.
#   build.sh [<driver patch>]   also builds driver_patched/ with the patch
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=/ros2_ws/build/_fake_franka
DRIVER_SRC=/ros2_ws/src/panda-driver-topics/ros2_panda_driver_topics

cmake -S "$HERE" -B "$OUT/lib" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$OUT/install" > /dev/null
cmake --build "$OUT/lib" -j"$(nproc)"
cmake --install "$OUT/lib" > /dev/null

cmake -S "$DRIVER_SRC" -B "$OUT/driver" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$OUT/install;${CMAKE_PREFIX_PATH}" \
    -DFranka_DIR="$OUT/install/lib/cmake/Franka"
cmake --build "$OUT/driver" -j"$(nproc)"
echo "Built: $OUT/driver/panda_driver_topics (linked to the FAKE libfranka)"

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
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="$OUT/install;${CMAKE_PREFIX_PATH}" \
        -DFranka_DIR="$OUT/install/lib/cmake/Franka"
    cmake --build "$OUT/driver_patched" -j"$(nproc)"
    echo "Built: $OUT/driver_patched/panda_driver_topics (with $(basename "$PATCH"))"
fi
