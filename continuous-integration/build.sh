#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?
ROOT_DIR=$(realpath "$SCRIPT_DIR/..") || exit $?

echo "Creating catkin workspace"
source /opt/ros/melodic/setup.bash || exit $?
mkdir -p ~/catkin_ws/src || exit $?
cd ~/catkin_ws || exit $?
catkin build || exit $?

echo "Adding link to the source folder"
ln -s "$ROOT_DIR" ~/catkin_ws/src || exit $?

for package in zivid_camera zivid_samples
do
    echo "Verify that $package is found by catkin list"
    catkin list --unformatted | grep -q $package || exit $?
done

echo "Installing dependencies"
rosdep update && rosdep install --from-paths src --ignore-src -r -y || exit $?

echo "Building zivid_ros"
catkin build || exit $?

echo "Installing Zivid API config file"
install -D "$SCRIPT_DIR"/ZividAPIConfigCPU.yml "$HOME"/.config/Zivid/API/Config.yml || exit $?

echo "Download and install zivid sample data (file camera)"
wget -q https://www.zivid.com/software/ZividSampleData.zip || exit $?
mkdir -p /usr/share/Zivid/data/ || exit $?
unzip ./ZividSampleData.zip -d /usr/share/Zivid/data/ || exit $?
rm ./ZividSampleData.zip || exit $?

echo "Running tests"
catkin run_tests || exit $?

echo "Check for test errors"
catkin_test_results || exit $?

echo Success! ["$(basename $0)"]
