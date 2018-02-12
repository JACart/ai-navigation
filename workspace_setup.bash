#!bin/bash
set -x
sudo apt-get install ros-kinetic-controller-manager
cd src
git clone https://github.com/weilannw/ackermann_vehicle-1.git
git clone https://github.com/ros-drivers/ackermann_msgs.git
cd ..
catkin_make
source $(pwd)/devel/setup.bash
echo source $(pwd)/devel/setup.bash >> ~/.bashrc
rosdep install --from-paths src --ignore-src
set +x
