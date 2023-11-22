# MAVROS
```
cd ~/catkin_ws
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
sudo bash -c "$install_geo"

catkin build

catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
echo "$catkin_ws_source" >> ~/.bashrc
eval $catkin_ws_source
```
# Cartographer
```
cd ~/catkin_ws/src
mkdir cartographer
cd cartographer
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git
```
```
rosdep install --from-paths . --ignore-src -r -y
cd cartographer/scripts
./install_abseil.sh
```
```
cd ~/catkin_ws
catkin build

```
# Move_base
```
cd ~/catkin_ws/src
sudo apt-get install ros-noetic-tf2-sensor-msgs
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/ros-planning/navigation_msgs
catkin build

```

# Robot_Localization
```
sudo apt-get install ros-noetic-robot-localization
```

# lib

### 
```
 sudo apt-get install ros-noetic-gmapping ros-noetic-amcl ros-noetic-move-base
 sudo apt-get install libgtest-dev libgmock-dev
```
