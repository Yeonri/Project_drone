# Install PX4_SITL and gazebo


## ** CMack update **
```
wget https://github.com/Kitware/CMake/releases/download/v3.19.8/cmake-3.19.8.tar.gz
tar zxf cmake-3.19.8.tar.gz && cd cmake-3.19.8
./bootstrap
make
sudo make install
```


### install PX4_Autopilot
```
cd /home/(user)/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

# 라즈베리파이에 설치시
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

### Update gazebo
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
```

### PX4_Simulation
```
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```
