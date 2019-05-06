#!/bin/bash

export DONT_RUN=1
export PX4_HOME_LAT=53.5450421
export PX4_HOME_LON=9.9957323
export PX4_HOME_ALT=5
export PX4_SIM_SPEED_FACTOR=1
# export model="iris"
export HEADLESS=1
# export PX4_SIM_MODEL="iris_fpv_cam"
export PX4_SIM_MODEL="typhoon_h480"
source "./Tools/setup_gazebo.bash" . ./build/px4_sitl_default
gzserver --verbose "./Tools/sitl_gazebo/worlds/${PX4_SIM_MODEL}.world" &

set +e
export rootfs="./build/px4_sitl_default/tmp/rootfs"
mkdir -p "$rootfs" 

./build/px4_sitl_default/bin/px4 ./ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t ./test_data
