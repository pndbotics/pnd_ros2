[TOC]
# pndbotics 机器人ros2支持
pndbotics SDK2基于cyclonedds实现了一个易用的机器人数据通信机制，应用开发者可以利用这一接口实现机器人的数据通讯和指令控制。
ROS2也使用DDS作为通讯工具，因此adam的底层可以兼容ros2，使用ros2自带的  msg 直接进行通讯和控制，而无需通过sdk接口转发。

# 环境配置
## 系统要求
测试过的系统和ros2版本
|系统|ros2 版本|
|--|--|
|Ubuntu 22.04|humble (推荐)|

## 安装 pndbotics 机器人ros2功能包

以下以ros2 humble为例，如需要其他版本的ros2，在相应的地方替换humble为当前的ros2版本名称即可：

ROS2 humble的安装可参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

ctrl+alt+T打开终端，克隆仓库：https://github.com/pndbotics/pnd_ros2
```bash
git clone https://github.com/pndbotics/pnd_ros2
```
其中
- **cyclonedds_ws** 文件夹为编译和安装 pndbotics 机器人ROS2 msg的工作空间，在子文件夹cyclonedds_ws/pndbotics/pnd_adam中定义了机器人状态获取和控制相关的ros2 msg。
- **example** 文件夹为 pndbotics 机器人 ROS2 下的相关例程。


## 安装 pndbotics 机器人ros2功能包

### 1. 安装依赖

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install libyaml-cpp-dev
```
### 2. 编译cyclone-dds
由于 pndbotics 机器人使用的是cyclonedds 0.10.2，因此需要先更改ROS2的dds实现。见：https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html

编译cyclonedds前请确保在启动终端时**没有**自动source ros2相关的环境变量，否则会导致cyclonedds编译报错。如果安装ROS2时在~/.bashrc中添加了 " source /opt/ros/humble/setup.bash "，需要修改 ~/.bashrc 文件将其删除：

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
``` 
在弹出的窗口中，注释掉ros2相关的环境变量，例如：
```bash
# source /opt/ros/humble/setup.bash 
```
在终端中执行以下操作编译cyclone-dds
```bash
cd ~/pnd_ros2/cyclonedds_ws/src
#克隆cyclonedds仓库
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
# 如果编译报错，尝试先运行：`export LD_LIBRARY_PATH=/opt/ros/humble/lib`
colcon build 
```

### 3. 编译功能包
编译好 cyclone-dds 后就需要 ros2 相关的依赖来完成 pndbotics 功能包的编译，因此编译前需要先 source ROS2 的环境变量。

```bash
source /opt/ros/humble/setup.bash #source ROS2 环境变量
colcon build #编译工作空间下的所有功能包
```