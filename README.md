# Low-cost SLAM System

This repository contains the source code and hardware design for a low-cost SLAM (Simultaneous Localization and Mapping) system. The system aims to provide a cost-effective solution for mapping environments and localizing within them.

## Hardware Design

This section contains model files for the chassis base, radar mounting base, and overall design schematic of the robot. You can find these files under the "Hardware Design" directory.

## MCU Code

The MCU (Microcontroller Unit) code section includes the project of the C30D(stm32).

## sys_ws

The sys_ws folder contains the source code for running on the Jetson Nano, including modules for controlling the motion of the robot and implementing SLAM functionality. The SLAM system is currently compatible with the fast-lio and lio-sam frameworks. Future updates will include compatibility with additional SLAM systems.

## Usage Guide

该部分提供了使用该 SLAM 系统的详细教程。包括系统组装说明、软件环境配置、系统启动和操作步骤等内容。通过本教程，您将能够快速上手并开始使用低成本 SLAM 系统进行环境建图和定位任务。
