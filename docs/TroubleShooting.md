# install PX4 

## bash ~/catkin_ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh
```
ERROR: pandas 2.0.3 has requirement numpy>=1.20.3; python_version < "3.10", but you'll have numpy 1.17.4 which is incompatible.
```
```
pip install numpy --upgrade

# 만약 ubuntu의 파이썬버전이 낮아서 설치가 되지 않을 때
# 파이썬 3.8을 설치 후 venv를 사용하여 가상환경을 activate 이후 설치

```

## 라즈베리파이에 PX4_SITL (시뮬레이션)을 설치할 경우 - (ARM 지원 X)
### Error
```
Installing NuttX dependencies
Reading package lists…
Building dependency tree…
Reading state information…
Package gcc-multilib is not available, but is referred to by another package.
This may mean that the package is missing, has been obsoleted, or
is only available from another source

E: Unable to locate package g+±multilib
E: Couldn’t find any package by regex ‘g+±multilib’
E: Package ‘gcc-multilib’ has no installation candidate
```
```
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

```
# catkin build
## MAVROS 설치 에러 libmavconn
```
CMake Error at /home/yeonri/catkin_ws/devel_isolated/libmavconn/share/libmavconn/cmake/libmavconnConfig.cmake:113 (message):
  Project 'libmavconn' specifies '/include' as an include dir, which is not
  found.  It does neither exist as an absolute directory nor in
  '/home/yeonri/catkin_ws/src/mavros/libmavconn//include'.  Check the issue
  tracker 'https://github.com/mavlink/mavros/issues' and consider creating a
  ticket if the problem has not been reported yet.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:76 (find_package)
  CMakeLists.txt:7 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/yeonri/catkin_ws/build_isolated/mavros/CMakeFiles/CMakeOutput.log".
See also "/home/yeonri/catkin_ws/build_isolated/mavros/CMakeFiles/CMakeError.log".
<== Failed to process package 'mavros': 
  Command '['/home/yeonri/catkin_ws/devel_isolated/mavros_msgs/env.sh', 'cmake', '/home/yeonri/catkin_ws/src/mavros/mavros', '-DCATKIN_DEVEL_PREFIX=/home/yeonri/catkin_ws/devel_isolated/mavros', '-DCMAKE_INSTALL_PREFIX=/home/yeonri/catkin_ws/install_isolated', '-G', 'Unix Makefiles']' returned non-zero exit status 1.

Reproduce this error by running:
==> cd /home/yeonri/catkin_ws/build_isolated/mavros && /home/yeonri/catkin_ws/devel_isolated/mavros_msgs/env.sh cmake /home/yeonri/catkin_ws/src/mavros/mavros -DCATKIN_DEVEL_PREFIX=/home/yeonri/catkin_ws/devel_isolated/mavros -DCMAKE_INSTALL_PREFIX=/home/yeonri/catkin_ws/install_isolated -G 'Unix Makefiles'

Command failed, exiting.
```
```
Errors     << libmavconn:cmake /home/yeonri/catkin_ws/logs/libmavconn/build.cmake.000.log
CMake Error in CMakeLists.txt:
  Imported target "Boost::system" includes non-existent path

    "/include"


```

### 1. 종속 패키지 cmake 설치 명령어
```
sudo apt-get install cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev
```
### 2. Cmake 설치
```
wget https://github.com/Kitware/CMake/releases/download/v3.19.8/cmake-3.19.8.tar.gz
tar zxf cmake-3.19.8.tar.gz && cd cmake-3.19.8
./bootstrap
make
sudo make install
```
# Cartographer 설치 Error


### 
```
Errors     << cartographer:make /home/yeonri/catkin_ws/logs/cartographer/build.make.000.log

Extension error:
Could not import extension sphinx.builders.latex (exception: cannot import name 'contextfunction' from 'jinja2' (/home/yeonri/.local/lib/python3.8/site-packages/jinja2/__init__.py))
make[2]: *** [docs/CMakeFiles/build_doc.dir/build.make:77: docs/CMakeFiles/build_doc] Error 2
make[1]: *** [CMakeFiles/Makefile2:3108: docs/CMakeFiles/build_doc.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
make: *** [Makefile:160: all] Error 2
cd /home/yeonri/catkin_ws/build/cartographer; catkin build --get-env cartographer | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -

..............................................................................
Failed     << cartographer:make                    [ Exited with code 2 ]     
Failed    <<< cartographer                         [ 1 minute and 46.7 seconds ]
```
### Sphinx 의존성 설치
```
pip install -U Sphinx
pip install Sphinx
```

# Move_base 패키지 설치 Error

### Could NOT find SDL (missing: SDL_LIBRARY SDL_INCLUDE_DIR)
```
sudo apt-get update
sudo apt-get install libsdl1.2-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
```

### Could not find a package configuration file provided by "tf2_sensor_msgs"
```
sudo apt-get install ros-noetic-tf2-sensor-msgs
```

### Could not find a package configuration file provided by "move_base_msgs
```
git clone https://github.com/ros-planning/navigation_msgs
```
