### Ctrl_pos_gazebo.launch 시뮬레이션 설정
실제 USB 연결: <param name="fcu_url" value="/dev/ttyUSB0:921600" />

시뮬레이션 기기 연결: <param name="fcu_url" value="udp://:14540@127.0.0.1:14557" />

### world export
export PX4_SITL_WORLD=

### lidar 센서 레이저 스캔 tf 추가
rosrun tf static_transform_publisher 0 0 0 0 0 0 map rplidar_link 100

### hector_slam에 적용되는 스캔 tf 추가
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link rplidar_link 100

export PX4_SITL_WORLD=office_small
### This is necessary to prevent some Qt-related errors (feel free to try to omit it)
export QT_X11_NO_MITSHM=1

### lidar 센서 레이저_스캔 노드 추가
<node pkg="tf" type="static_transform_publisher" name="map_rplidar_link_broadcaster" args="0 0 0 0 0 0 map rplidar_link 10"/>

### lidar 센서 레이저 스캔 tf 추가
rosrun tf static_transform_publisher 0 0 0 0 0 0 map rplidar_link 100

### hector_slam의 mapping_default.launch 파일 수정
laser_frame이 default로 되도록 수정
<arg name="base_frame" default="laser_frame"/>
<arg name="odom_frame" default="laser_frame"/>

### frame 확인
rosrun tf view_frames

### [ERROR} no mapping for sensor
(apm)px4_pluginlists.yaml 에 "- distance_sensor" 추가

### ctrl_traj_gazebo gedit
gedit /root/catkin_ws/src/modudculab_ros/launch/ctrl_traj_gazebo.launch
### amcl python3 pykdl 종속성 오류./start
apt-get install liborocos-kdl-dev

************ ros move_base를 설치할 때 catkin_ws에 가서 빌드해야 됨
```
cd ~/catkin_ws/src
git clone https://github.com/PX4/PX4-Autopilot
git clone https://github.com/PX4/PX4-SITL_gazebo-classic
git clone https://github.com/mavlink/mavlink-gbp-release
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/ros/common_msgs
git clone https://github.com/ros-planning/navigation_msgs
git clone https://github.com/jsk-ros-pkg/orocos_kinematics_dynamics_python3
git clone https://github.com/paulbovbel/frontier_exploration
sudo apt-get install python3-sip
sudo apt-get install sip-dev python3-sip-dev
```

cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
source ~/catkin_ws/devel/setup.bash

sudo apt-get install ros-noetic-tf2-sensor-msgs
