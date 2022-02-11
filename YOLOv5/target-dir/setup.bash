mkdir -p /ros2_ws/src
cd /ros2_ws/src
git clone --recursive https://github.com/Ar-Ray-code/YOLOv5-ROS.git
git clone https://github.com/Ar-Ray-code/movie_publisher.git -b foxy

# build ros2_ws
source /opt/ros/foxy/setup.bash
cd /ros2_ws
colcon build --symlink-install

source /ros2_ws/install/local_setup.bash