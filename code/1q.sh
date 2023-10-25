cd /home/yeonri/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export QT_X11_NO_MITSHM=1
roslaunch gazebo_ros empty_world.launch world_name:=/home/yeonri/catkin_ws/src/Project_drone/code/test1.world
