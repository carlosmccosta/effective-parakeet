#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?
ROOT_DIR=$(realpath "$SCRIPT_DIR/..") || exit $?

function error_exit {
    echo "$2" >&2
    exit "${1}"
}

echo "Creating catkin workspace"
source /opt/ros/melodic/setup.bash || exit $?
mkdir -p ~/catkin_ws/src || exit $?
cd ~/catkin_ws || exit $?
catkin build || exit $?

echo "Adding link to the source folder"
ln -s $ROOT_DIR ~/catkin_ws/src || exit $?

echo "Verify that packages are found by catkin list"
catkin list --unformatted | grep -q zivid_camera || error_exit $? "zivid_camera not found"
catkin list --unformatted | grep -q zivid_samples || error_exit $? "zivid_samples not found"

echo "Installing dependencies"
rosdep update && rosdep install --from-paths src --ignore-src -r -y || exit $?

echo "Building zivid_ros"
catkin build || exit $?

# TODO find out how to handle OpenCL in docker, so the tests can be run

# echo "Running tests"
# catkin run_tests || exit $?

# echo "Check for test errors"
# catkin_test_results || exit $?

echo Success! ["$(basename $0)"]
