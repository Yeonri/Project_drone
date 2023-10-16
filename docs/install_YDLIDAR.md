# 순서
- 1. install YDLidar-SDK : https://github.com/YDLIDAR/YDLidar-SDK
  2. install YDLidar_ros_driver : https://github.com/YDLIDAR/ydlidar_ros_driver/tree/master

# Install YDLidar-SDK

## 필수 패키지 설치
```
sudo apt install cmake pkg-config python3 swig python3-pip
```

## YDLIDAR-SDK install
```
cd ~/catkin_ws/src
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir -p /build
cd build
cmake ..
make
sudo make install
```

### YDLIDAR_ROS_driver install
```
cd ~/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
chmod 0777 ydlidar_ros_driver/startup/*
cd ~/catkin_ws && catkin_make
```

### 런치 파일 포트 설정 (YDlidar sensor을 연결한 이후)
```
# 연결된 포트 확인
dmesg | grep tty
# 연결된 포트 사용 권한 부여
sudo chmod a+rw /dev/ttyUSB0

# 포트 번호에 맞게 런치 파일 내용 수정
gedit ~/catkin_ws/src/ydlidar_ros_driver/launch/X2.launch

<param name="port"         type="string" value="/dev/ttyUSB0"/>
```

### YDLidar launch
```
roslaunch ~/catkin_ws/src/ydlidar_ros_driver/launch/X2.launch
```
