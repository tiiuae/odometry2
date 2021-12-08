#!/bin/bash

set -eo pipefail

mod_dir=${1}

cd ${mod_dir}

echo "[INFO] Get package dependencies."
# Dependencies from fog-sw repo
if [ -e ${mod_dir}/ros2_ws/src ]; then
    echo "[INFO] Use dependencies from fog_sw."
    pushd ${mod_dir}/ros2_ws > /dev/null
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    echo "[INFO] Use dependencies from local repository."
    mkdir -p ${mod_dir}/deps_ws/src
    pushd ${mod_dir}/deps_ws > /dev/null
    vcs import src < ${mod_dir}/underlay.repos
    rosdep install --from-paths src -r -y --rosdistro ${ROS_DISTRO}
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

rosdep_out=$(rosdep check -v --from-paths src 2>&1 | grep "resolving for resources" )
ALL_PKGS=$(echo $rosdep_out | sed 's/.*\[\(.*\)\].*/\1/' | tr ',' '\n' | tr -d ' ')
echo "[INFO] All packages: $(echo $ALL_PKGS|tr '\n' ' ')"
PKGS_TO_BUILD=""
pushd src > /dev/null
for pkg_name in ${ALL_PKGS}; do
    pkg_name=$(echo ${pkg_name} | sed 's/\/$//')
    if ! ros2 pkg list | grep ${pkg_name} 1> /dev/null 2>&1; then
        PKGS_TO_BUILD="${PKGS_TO_BUILD} ${pkg_name}"
    fi
done
popd > /dev/null

echo "[INFO] Build package dependencies."
colcon build --packages-select ${PKGS_TO_BUILD}
popd > /dev/null