# Install Cartorgrapher

### 필요 라이브러리 설치
```
sudo apt-get install ninja-build stow
```


### Git Clone
```
cd ~/catkin_ws/src
mkdir cartographer
cd cartographer
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git
```


### 의존 라이브러리 설치
```
rosdep install --from-paths . --ignore-src -r -y
```

### Install abseil
```
cd catrographer/scripts
./install_abseil.sh
```

### Catkin build
```
cd ~/catkin_ws
catkin build
```
