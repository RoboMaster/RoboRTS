# RoboRTS
[![Build Status](https://travis-ci.org/RoboMaster/RoboRTS.svg?branch=master)](https://travis-ci.org/RoboMaster/RoboRTS)
[![Gitter](https://badges.gitter.im/RoboMaster/RoboRTS.svg)](https://gitter.im/RoboMaster/RoboRTS?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

RoboRTS is an open source software stack for Real-Time Strategy research on mobile robots, developed by RoboMaster. The Real-Time Strategy research that we focus on is multi-agent deep reinforcement learning based decision making.

The motivation for this project is [RoboMaster AI Challenge](https://icra2018.org/dji-robomaster-ai-challenge/). In this robot challenge, multiple robots should fight with each other on a game field automatically. It would be very convenient to have a unified framework for them to integrate hardware components and implement algorithms.

A modern mobile robot usually contains a simple low-level computing platform runs RT OS (Real Time Operation System) with restricted computing power but real-time output, and a strong high-level computing platform runs non-RT OS (such as Linux and Windows) which dedicates to computational intensive algorithm execution. In our system, the low-level computing platform is MCU (micro-controller), and the high-level computing platform is called the on-board computer. 

The framework of RoboRTS consists two parts: autonomous mobile robot layer and intelligent decision-making layer.

<img src="docs/images/system.png" style="zoom:80%;display: inline-block; float:middle"/>

The autonomous mobile robot layer alone can let a robot to demonstrate a certain level of intelligence. On its on-board computer runs perception, planning, decision modules, each of which contains some conventional algorithms like A* search and Lidar mapping. On its MCU, an RT low-level robot controller is implemented to govern the robot driving system. This README mainly explains the structure and functionality of this layer.

**TODO:** Intelligent decision-making layer includes a multi-agent decision-making framework and a game simulator, it will be released in the future.

## Get started

You can build a mobile robot described in [hardware_setup](docs/hardware_setup.md) document and setup embedded system mentioned in [RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware) repository.

### Software Requirements

- ROS kinetic (Ubuntu 16.04)
- OpenCV 3.0.0 or higher
- CUDA(Optional)
- PCL
- G2O
- SuiteSparse
- Google Protocol Buffers 2.6
- Google Glog
- CMake 3.1

### Setup Software 

Please follow this [guide](docs/roborts_setup_guide.md) step by step. 

On Nvidia Jetson TX2, please follow this [tutorial(Chinese)](docs/setup_on_tx2.md) for more details.

## Documents

* [Hardware setup](docs/hardware_setup.md): Instruction to build a RoboMaster infantry robot.
* [Software architecture](docs/software_architecture.md): The software architecture of RoboRTS framework.
* [Protocol](https://github.com/RoboMaster/RoboRTS-Firmware/blob/master/Doc/protocol/readme.md): The protocol between the on-board computer and the MCU.
* [Tutorial](): Additional tutorials will be released in the future.

## Copyright and License

RoboRTS is provided under the [GPL-v3](COPYING).
