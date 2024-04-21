# BT_Sample PKG

![1](./fig/1.png)

## Table of Contents
- [BT\_Sample PKG](#bt_sample-pkg)
  - [Table of Contents](#table-of-contents)
  - [Behavior Tree Setup](#behavior-tree-setup)
    - [Install Groot 2](#install-groot-2)
    - [Launch Groot 2](#launch-groot-2)
    - [Install BehaviorTree.CPP (Optional)](#install-behaviortreecpp-optional)
  - [Build Behavior Tree Sample PKG](#build-behavior-tree-sample-pkg)
  - [Run Behavior Tree Sample PKG](#run-behavior-tree-sample-pkg)
    - [Groot2 Launch](#groot2-launch)
    - [Behavior Tree Sample PKG Launch](#behavior-tree-sample-pkg-launch)

## Behavior Tree Setup

### Install Groot 2

`.run` 파일을 설치 후 권한을 부여 후 실행

```bash
wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.5.2-linux-installer.run
chmod +x Groot2-v1.5.2-linux-installer.run 
./Groot2-v1.5.2-linux-installer.run
```

### Launch Groot 2

```bash
cd ~/Groot2/bin
./groot2
```

> 다음 명령어를 통해 groot 로 단축키를 만들어 사용할 수 있습니다. 

```bash
echo "alias groot='cd ~/Groot2/bin ; ./groot2'" >> ~/.bashrc
```

### Install BehaviorTree.CPP (Optional)

전역적으로 사용할 수 있도록 설치합니다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP
sudo apt install libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
cd ~/ros2_ws/src/BehaviorTree.CPP
mkdir build && cd build
cmake ..
sudo make -j8 install
```

## Build Behavior Tree Sample PKG

```bash
cd ~/ros2_ws/src
git clone https://github.com/AuTURBO/bt_template.git
cd ~/ros2_ws
colcon build
```

## Run Behavior Tree Sample PKG

### Groot2 Launch

```bash
~/Groot2/bin/groot2
```

### Behavior Tree Sample PKG Launch

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run bt_sample bt_node
```
