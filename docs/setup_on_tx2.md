RoboRTS在Nvidia Jetson TX2上应用时的一些配置。

## RoboRTS On TX2

### Nvidia Jetson TX2调试常用软件与脚本

PS：在调试TX2过程出现任何无法解决的问题，请记录复现条件、问题，并及时反馈给工程师。

- **NVPModel和Jetson Clocks**

  - NVPModel

    五种模式来分配cpu核数，cpu和gpu的最大频率，TX2默认模式是只开4个`CPU`, 所以你如果要打开其最大性能的话,那就试试`nvpmodel`。


| MODE |   MODE NAME    | DENVER 2 | FREQUENCY | ARM A57 | FREQUENCY | GPU FREQUENCY |
| :--: | :------------: | :------: | :-------: | :-----: | :-------: | :-----------: |
|  0   |     Max-N      |    2     |  2.0 GHz  |    4    |  2.0 GHz  |   1.30 Ghz    |
|  1   |     Max-Q      |    0     |           |    4    |  1.2 GHz  |   0.85 Ghz    |
|  2   | Max-P Core-All |    2     |  1.4 GHz  |    4    |  1.4 GHz  |   1.12 Ghz    |
|  3   |   Max-P ARM    |    0     |           |    4    |  2.0 GHz  |   1.12 Ghz    |
|  4   |  Max-P Denver  |    2     |  2.0 GHz  |    0    |           |   1.12 Ghz    |

更改模式

```bash
sudo nvpmodel -m [mode]
```

查看当前模式

```bash
sudo nvpmodel -q --verbose
```
  - jetson_clocks.sh

    更改时钟频率脚本，主要为了开启最大性能，jetpack会在用户的目录安装这个脚本

    ```bash
    $ ./jetson_clocks.sh
    ```

  - 开启最大性能

    ```bash
    echo "Enabling fan for safety..."
    if [ ! -w /sys/kernel/debug/tegra_fan/target_pwm ] ; then
    echo "Cannot set fan -- exiting..."
    fi
    echo 255 > /sys/kernel/debug/tegra_fan/target_pwm
    echo "Enabling 6 cpus and gpu with maximum frequency"
    nvpmodel -m 0
    ./jetson_clocks.sh
    ```

- **网速测试和连接**

  - TX2上以太网带宽和速度测试

    ``` bash
    $ sudo apt-get install iperf
    #start server
    $ sudo iperf -s
    #start client
    $ sudo iperf -c 192.168.1.xxx -i 5
    ```

  - TX2上WiFi测试

    - 关闭wlan0节能模式

      ```bash
      iw dev wlan0 set power_save off #- to disable power save and reduce ping latency.
      iw dev wlan0 get power_save  # will return current state.

      ```

    - 查看WiFi的RSSI

      ```bash
      watch -n 0.1 iw dev wlan0 link
      ```

      ​

  - 远程连接：寻找局域网内的tx2设备并启用ssh连接

  ```bash
  dev () 
  { 	
  	# 寻找局域网内的tx2 ： sudo arp-scan --interface=eth0 192.168.1.0/24 | grep NVIDIA 
  	# eth0是以太网，如果用wifi就找对应的wifi接口，例如wlp4s0或者wlan0等
      local ip="$(sudo arp-scan --interface=eth0 192.168.1.0/24 | grep NVIDIA | sed -n $1p | cut -d'	' -f 1)";
      echo "ssh into $ip";
      ssh nvidia@$ip
  }
  ```

- **其他常用软件**


```bash
#terminal
sudo apt-get install terminator -y
#git
sudo apt-get install git
#cmake
sudo apt-get install cmake -y
#vim
sudo apt-get install vim -y
#top
sudo apt-get install htop
```



### 安装配置软件依赖

1. 安装ROS(ros-kinetic-ros-base)
   - 参见ROS Wiki的[安装教程](http://wiki.ros.org/kinetic/Installation/Ubuntu)或者[JetsonHacks Github Repo](https://github.com/jetsonhacks/installROSTX2.git)的脚本，特别要注意更新本地Repo否则可能缺乏依赖：
   ```bash
   sudo apt-add-repository universe
   sudo apt-add-repository multiverse
   sudo apt-add-repository restricted
   sudo apt-get update
   # Install Robot Operating System (ROS) on NVIDIA Jetson TX2
   # Maintainer of ARM builds for ROS is http://answers.ros.org/users/1034/ahendrix/
   # Information from:
   # http://wiki.ros.org/kinetic/Installation/UbuntuARM
   ```
   - 然后就可以开始安装ROS：
   ```bash
   # Setup sources.lst
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   # Setup keys
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
   # Installation
   sudo apt-get update
   sudo apt-get install ros-kinetic-ros-base -y
   # Add Individual Packages here
   # You can install a specific ROS package (replace underscores with dashes of the package name):
   # sudo apt-get install ros-kinetic-PACKAGE
   # e.g.
   # sudo apt-get install ros-kinetic-navigation
   #
   # To find available packages:
   # apt-cache search ros-kinetic
   # 
   # Initialize rosdep （可选）
   # sudo apt-get install python-rosdep -y
   # Certificates are messed up on the Jetson for some reason
   sudo c_rehash /etc/ssl/certs
   # Initialize rosdep
   sudo rosdep init
   # To find available packages, use:
   rosdep update
   # Environment Setup - Don't add /opt/ros/kinetic/setup.bash if it's already in bashrc
   grep -q -F 'source /opt/ros/kinetic/setup.bash' ~/.bashrc || echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   # Install rosinstall （可选）
   # sudo apt-get install python-rosinstall -y
   ```
   - 其他ROS环境变量配置
   ```bash
   # Network Configuration
   # grep -q -F ' ROS_MASTER_URI' ~/.bashrc ||  echo 'export ROS_MASTER_URI=http://localhost:11311' | tee -a ~/.bashrc
   # grep -q -F ' ROS_IP' ~/.bashrc ||  echo "export ROS_IP=$(hostname -I)" | tee -a ~/.bashrc
   # grep -q -F ' ROS_HOSTNAME' ~/.bashrc ||  echo "export ROS_HOSTNAME=$(hostname -s)" | tee -a ~/.bashrc
   echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```
2. 安装ROS所需第三方依赖包，以及SuiteSparse，Glog，Protobuf等其他依赖
   ```bash
   $ sudo apt-get install -y ros-kinetic-opencv3 ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-stage-ros ros-kinetic-map-server ros-kinetic-laser-geometry ros-kinetic-interactive-markers ros-kinetic-tf ros-kinetic-pcl-* ros-kinetic-libg2o protobuf-compiler libprotobuf-dev libsuitesparse-dev libgoogle-glog-dev ros-kinetic-rviz
   ```
### 下载配置RoboRTS-ros进行仿真
1. 克隆并编译RoboRTS
   ```bash
   #切换至clone下来的RoboRTS_ros的目录内
   cd RoboRTS_ros
   #先编译自定义msg文件
   catkin_make messages_generate_messages
   #编译全部源码
   catkin_make
   ```
2. 配置RoboRTS环境变量
    ```bash
    cd RoboRTS_ros
    #注意，如果使用的是zsh，请source对应的zsh文件
    source devel/setup.bash
    #如果使用Gazebo仿真，需要配置plugin的path
    export GAZEBO_PLUGIN_PATH=PATH_TO_RoboRTS_ros/devel/lib:$GAZEBO_PLUGIN_PATH
    ```
3. 运行仿真
    ```bash
    #运行基于Stage仿真器的navigation仿真
    roslaunch roborts navigation_stage.launch

    #运行基于Gazebo仿真器的navigation仿真
    roslaunch roborts navigation_gazebo.launch

    #如需查看Gazebo的GUI界面
    rosrun gazebo_ros gzclient
    ```
### 实车测试注意事项
1. 根据硬件接口（串口，USB或者ACM）来配置/etc/udev/rules.d中的udev文件，分别实现rplidar和stm32串口的设备绑定：
   首先插入USB设备，lsusb可以查看Vendor和Product的 ID，然后创建并配置/etc/udev/rules.d/rplidar.rules文件
   ```bash
   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
   ```
   同理rmserial.rules文件， 注意`SYMLINK+="rm/serial"`。然后重新加载并启动udev服务。可能需要重新插拔设备后生效。
   ```bash
   $ sudo service udev reload
   $ sudo service udev restart
   ```
   而配置多个相同型号的相机的会麻烦一些，由于Vendor和Product的ID是一样的，因此要查看每个相机具体的特性
   ```bash
   udevadm info --attribute-walk --name=/dev/video0
   ```
   一般可以用串口号不同（serial）作为属性来区分每个相机，例如：
   ```bash
   SUBSYSTEM=="usb", ATTR{serial}=="68974689267119892", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
   SUBSYSTEM=="usb", ATTR{serial}=="12345698798725654", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
   ```
   如果是廉价相机，可能串口号也相同，可以由连接HUB的物理端口不同（KERNEL或KERNELS绑定）来配置，例如：
   ```bash
   SUBSYSTEM=="usb", KERNEL=="2-3", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
   SUBSYSTEM=="usb", KERNEL=="2-4", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
   ```

2. 调试期间，需要配置多机器ROS通信，参考上面ROS其他环境变量的配置

3. **Notice**:

    - 仿真中雷达的frame和实车中可能不相同，注意修改对应参数
    - 各个模块参数都在对应模块文件夹的config文件夹中的prototxt文件内，各个参数的定义可以参考各模块API文档
    - `modules/decision/config/decision.prototxt`文件会配置一开始的巡逻点，为了方便测试最开始还是都注释掉比较好
    - `modules/perception/localization/amcl/config/amcl.prototxt`文件会配置一开始的默认初始位置

4. 运行实际测试脚本    
  ```bash
  roslaunch roborts 2v2game.launch
  ```


### 自动启动脚本

[FYI](https://magiccvs.byu.edu/wiki/#!computers/systemd.md)
