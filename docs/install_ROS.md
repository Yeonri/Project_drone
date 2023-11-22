# Install_ROS

## 사용자 권한 설정
```
vim /etc/sudoers

# i 눌러서 insert 모드 후 root 밑에 작성
user ALL=(ALL:ALL):ALL

# esc 누른 이후 :wq! 로 저장 후 나가기
:wq! 
```

## 1. ROS 서버에서 package를 받아오고 cURL를 설정한 이후 update
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
```

## 2. ROS Noetic 설치
```
sudo apt-get install ros-noetic-desktop-full
```

## 3. 환경변수 설정


###  ~/.bashrc에 setup.bash 추가
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### ROS 종속 패키지 설치
```
sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-roslaunch  python3-catkin-tools
```

### init & rosdep update
```
sudo rosdep init
rosdep update
```

### make workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin init
wstool init src
catkin build
```

### ./bashrc setting
```
gedit ~/.bashrc
# 맨 마지막 줄에 해당 내용 추가

source ~/catkin_ws/devel/setup.bash
alias eb =‘nano ~/.bashrc'
alias sb ='source ~/.bashrc'
alias cw ='cd ~/catkin_ws'
alias cs ='cd ~/catkin_ws/src'
alias cm ='cd ~/catkin_ws && catkin_make'
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

### install qtcreator
```
sudo apt-get install qtcreator
```

### roscore
```
roscore
```
