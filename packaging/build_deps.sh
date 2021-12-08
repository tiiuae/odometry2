#!/bin/bash

set -eo pipefail

mod_dir=${1}

cd ${mod_dir}

echo "[INFO] Get package dependencies."
# Dependencies from fog-sw repo
if [ -e ${mod_dir}/ros2_ws/src ]; then
    pushd ${mod_dir}/ros2_ws > /dev/null
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    mkdir -p ${mod_dir}/deps_ws/src
    pushd ${mod_dir}/deps_ws > /dev/null
    vcs import src < ${mod_dir}/underlay.repos
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi
echo "[INFO] Build package dependencies."
colcon build --packages-select fog_msgs px4_msgs
popd > /dev/null
