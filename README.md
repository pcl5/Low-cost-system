<!--
 * @Description: TM4C
 * @file: ${filename}
 * @brief: ${BRIEF}
 * @Version: 1.0
 * @Autor: PCL
 * @Date: 2024-04-02 22:22:45
 * @Code Reference: cbr wgj wx
 * @LastEditTime: 2024-04-02 23:36:46
-->
# A Low-cost 3D SLAM System

This work is based on our paper. This repository contains the source code and hardware design for a low-cost SLAM (Simultaneous Localization and Mapping) system. The system aims to provide a cost-effective solution for mapping environments and localizing within them.

**Author:** Conglin Pang, Liqing Zhou and Xianfeng Huang

## Hardware Design

This section contains model files for the chassis base, radar mounting base, and overall design schematic of the robot. You can find these files under the "Hardware Design" directory.

## MCU Code

The MCU (Microcontroller Unit) code section includes the project of the C30D(stm32), for detailed documentation refer to [wheeltec](https://github.com/wheeltec/docs).

## SYS_WS

The sys_ws folder contains the source code for running on the Jetson Nano, including modules for controlling the motion of the robot and implementing SLAM functionality. The SLAM system is currently compatible with the fast-lio and lio-sam frameworks. Future updates will include compatibility with additional SLAM systems.

## Usage Guide

The code has been tested only with Ubuntu 18.04 (ROS-melodic) and 20.04 (ROS-noetic).

1. Installing ROS controling-model on Jetson Nano
We print the base plate using carbon brazed material (in base and base2 files), you can see the detailed assembly details of the whole vehicle, including Mid-360, Battery and others, in the car_base file.
```
Low-cost-system/
  hardware design/
    base.SLDPRT
    base2.SLDPRT
    car_base.SLDASM
```
If necessary, the base or base2 file holes can be modified in solidworks to suit your hardware requirements.

2. Installing ROS controling-model on Jetson Nano

Before starting, please connect the Uart1 of STM32 to the host computer with a usb cable, and make sure that you have fixed the serial port number and empowered the serial port.

- Installing ros-serial:
```
sudo apt-get update
sudo apt-get install ros-<ros_version>-serial
```

- Installing ROS controling-model
```
cd
git clone https://github.com/pcl5/Low-cost-system.git
cd sys_ws
catkin_make
source ./devel.bash
```
After that, you can control whole vehicle by Jetson nano.

3. Installing Livox-SDK2

Livox mid360 requires livox-SDK2 instead of livox-SDK and the corresponding livox_ros_driver2. You can find the installation instructions in the [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2/tree/master).
- Install the Cmake using apt:
```
sudo apt install cmake
```
- Install the Livox-SDK2:
```
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```



