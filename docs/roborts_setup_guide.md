## Setup Instruction

### Contents

1. Install Prerequisites
2. Clone repository and Build
3. Run Example in Simulation

#### 1. Install Prerequisites

1. Install [Ubuntu 16.04 64-bit](http://releases.ubuntu.com/16.04/) on your x86 PC or use [JetPack](https://developer.nvidia.com/embedded/jetpack) for system installation on [Nvidia Jetson TX2](https://developer.nvidia.com/embedded/develop/hardware).

2. Install required packages

   ```bash
   sudo apt install git vim openssh-client openssh-server build-essential
   ```

   Recommended but not required packages

   - cutecom: for serial port communication test.

3. Install ROS.

   Install `ros-kinetic-desktop-full` on your PC or install `ros-kinetic-base`. You cloud follow [the installing and configuring ROS environment tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) on ROS Wiki.

4. Install RoboRTS build dependencies.

   ```bash
   sudo apt-get install ros-kinetic-opencv3 ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-stage-ros ros-kinetic-map-server ros-kinetic-laser-geometry ros-kinetic-interactive-markers ros-kinetic-tf ros-kinetic-pcl-* ros-kinetic-libg2o protobuf-compiler libprotobuf-dev libsuitesparse-dev libgoogle-glog-dev ros-kinetic-rviz
   ```

#### 2. Clone repository and Build

```bash
# Make dir for workspace
mkdir -p $HOME/roborts_ws/src

# Clone this repository
cd $HOME/roborts_ws/src
git clone https://github.com/RoboMaster/RoboRTS.git

# Build RoboRTS
cd $HOME/roborts_ws
catkin_make messages_generate_messages
catkin_make
```

#### 3. Run Example in Simulation

```bash
source $HOME/roborts_ws/devel/setup.bash
roslaunch roborts navigation_stage.launch
```

