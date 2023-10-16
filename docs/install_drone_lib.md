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


## ERROR: plain Cmake Error

```
# 만약 catkin_make를 사용하여 plain cmake error가 존재할 경우
# catkin_make_isolated
# 본인은 catkin_make가 안되서 catkin clean 이후 catkin build로 변경함
```
