#!bin/bash
set -x
sudo apt-get install ros-kinetic-controller-manager
catkin_make
source $(pwd)/devel/setup.bash
echo source $(pwd)/devel/setup.bash >> ~/.bashrc
rosdep install --from-paths src --ignore-src
set +x
