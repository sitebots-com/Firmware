#!/bin/bash

export HEADLESS=1
source "./Tools/setup_gazebo.bash" . ./build/px4_sitl_default

set +e
export rootfs="./build/px4_sitl_default/tmp/rootfs"
mkdir -p "$rootfs" 

./build/px4_sitl_default/bin/px4 ./ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t ./test_data
