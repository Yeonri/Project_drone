# terminal_1
roscore

# terminal_2
```
cd /home/yeonri/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export QT_X11_NO_MITSHM=1
roslaunch gazebo_ros empty_world.launch world_name:=/home/yeonri/catkin_ws/src/Project_drone/code/test1.world
```
# terminal_3
```
cd /home/yeonri/PX4-Autopilot
make px4_sitl_default gazebo-classic_iris_rplidar
```
# terminal_4
```
roslaunch mavros px4.launch fcu_url:="udp://:14550@localhost:11345"
```
# 스캔 tf 추가
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link rplidar_link 100
```

# gone2.launch
```
roslaunch gone2.launch
```
# test4.py
```
python test4.py
```

# terminal_5
```
rosrun mavros mavsafety arm fcu_url:=udp://:14540@localhost:11345
rosrun mavros mavsys mode -c OFFBOARD
```

sudo nautilus
