### These instructions are used for Ubuntu 20.04. They could be used in WSL2 or as a dual boot. But it's highly recommended to be used as a dual boot.


# Setup of Companion Computer / PC
## ROS Noetic
Refernce for ROS installation: [ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Checks:
Check for any updates.

```
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt install git -y
```
### Setup your sources.list:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up your keys:
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### Installation:
First, make sure your Debian package index is up-to-date:
```
sudo apt update -y
```
Desktop-Full Install:
```
sudo apt install ros-noetic-desktop-full -y
```
### Environment setup:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Dependencies for building packages:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y
```
### Initialize rosdep:
```
sudo rosdep init
rosdep update
```

## Mavros
This is Mavros Source Installion. Refernces ([PX4 docs here](https://docs.px4.io/main/en/ros/mavros_installation.html#source-installation) and [Mavros repo on Github here](https://github.com/mavlink/mavros/tree/master/mavros#source-installation)). Note that they're using ROS Kinetic in the refernces and we are using ROS Noetic. The next instructions work for ROS Noetic.

### Install Deps:
```
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```
### Create ROS workspace in your home directory:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```
### Install MAVLink:
```
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
```
### Install MAVROS: get source (upstream - released):
```
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```
### Create workspace & deps:
```
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```
### Install GeographicLib datasets:
```
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
### Build source:
```
cd ~/catkin_ws
catkin build
```
Make sure that you use setup.bash from workspace. Else rosrun can't find nodes from this workspace.
```
source devel/setup.bash
```
### Fetch our work:
1. Mavros

```
cd ~/catkin_ws/src/mavros/
git remote add upstream https://github.com/Mu99-M/mavros.git
git fetch upstream
git checkout MK_ROS
```

2. Mavlink

```
cd ~/catkin_ws/src/mavlink/
git remote add upstream https://github.com/Mu99-M/mavlink-gbp-release.git
git fetch upstream
git checkout MK_MAVLINK
```
### Rebuild catkin_ws:
```
cd ~/catkin_ws
catkin build
```

## offb Node
This node has 3 jobs.
1. Switch to offboard mode and checks if it's disconnected to try switching again.
2. Arm the vehicle and checks if it's disarmed to try arming again.
3. Publishing values to `ActuatorMotors` topic.

### Installing:
```
cd ~/catkin_ws/src/
git clone -b master https://github.com/Mu99-M/offb.git --recursive
cd ~/catkin_ws
catkin build
```
Now everthing is ready to be used from companion computer side.


# QGroundControl (QGC)
### Deps Before Installing:
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```
After that you should sign out and sign in again to enable the change to user permissions. Please save any unsaved work.

### To install QGroundControl:
1. Download [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage).
2. Install (and run) using the terminal commands:
```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  # (or double click)
```


# Setup of Pixhawk / PX4

## Firmware Installation
You can either use our pre-buit firmware files or build the firmware from source. Both procedures are describe below:

### Loading pre-built firmware to Pixhawk:

You can find the files for pre-built firmware on this [link](https://drive.google.com/drive/folders/1ms6OHxm7kN3-U2eWSx3bNYO67roZdURy?usp=drive_link). Download the firmware suitabe for your FMU version and upload it to your pixhawk using QGroundControl as explained in this [guide](https://docs.qgroundcontrol.com/master/en/SetupView/Firmware.html).


### Building custom firmware from source:
#### Deps Before Cloning:
These installations solved all of the errors I faced.
```
sudo apt install python3-pip -y
pip3 install kconfiglib
sudo apt install gcc-arm-none-eabi -y
pip3 install --user jinja2
pip3 install --user jsonschema
```

#### Cloning PX4:
Clone the branch `MK` from `Mu99-M/PX4-Autopilot` in home directory.
```
cd ~
git clone -b MK https://github.com/Mu99-M/PX4-Autopilot.git --recursive
```
#### Install gstreamer for SITL:
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```

#### Building PX4:
Now you can build PX4 again for SITL without any problem. The final result of the following command should open gazebo with the drone in it.
```
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
To build for Pixhawk 4 (FMUv5):
```
cd ~/PX4-Autopilot
make px4_fmu-v5_default
```
For any other version check [this link](https://docs.px4.io/main/en/dev_setup/building_px4.html#building-for-nuttx).

## Setup ethernet communication with pixhawk (Optional):
You can refer to this [guide](https://docs.px4.io/main/en/advanced_config/ethernet_setup.html) to setup ethernet communication with you pixhawk if your pixhawk supports this (e.g. pixhawk 5x or 6x). If using the pixhawk cm4 baseboard with raspberry pi, you can refer to this [guide](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-rpi-cm4-baseboard/ethernet-connection). The process to setup ethernet comunication with pixhawk is briefly explained below in the report in `1. Using Ethernet` subsection.


## PX4 parameter configurations
To configure the PX4 parameters correctly, you can directly upload a pre-configures file to your pixhawk / SITL. You can also configure the needed parameters manually as describe later in this guide.

### Loading pre-configured parameters set
You can find pre-configured parameters files on [this link](https://drive.google.com/drive/folders/1zgydF3k8lJNwXT9q2XfnUk0Gt9lqb0XE?usp=drive_link). You can upload the pre-configured parameter file to pixhawk using QGroundControl as described in [this link](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html)

### Manually configure paramaters (Optional)
You can change px4 parameters from QGroundControl as described [here](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html) The major needed parameter changes are described below:


#### Configuring the MAVLINK instance

##### 1. Using Ethernet

If you have already configured a static IP for your pixhawk and companion computer as explained in this [guide](https://docs.px4.io/main/en/advanced_config/ethernet_setup.html), you can skip to step 8.
1. Connect the pixhawk to QGC using a USB-C cable.
2. Open QGroundcontrol > Analyze Tools > MAVLink Console.
3. Enter the following commands: \
   echo DEVICE=eth0 > /fs/microsd/net.cfg \
                                        echo BOOTPROTO=static >> /fs/microsd/net.cfg\
                                        echo IPADDR=192.168.0.4 >> /fs/microsd/net.cfg\
                                        echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg\
                                        echo ROUTER=192.168.0.1 >>/fs/microsd/net.cfg\
                                        echo DNS=192.168.0.1 >>/fs/microsd/net.cfg\
   The IPADDR could be chosen by the user. Our default configuration use `IPADDR=192.168.0.4` as the IP address of the pixhawk. The pixhawk subnet should match the subnet of the companion computer.
4. Connect the pixhawk to an external power source, and connect the ethernet cable to the companion computer / PC.
5. If the companion computer / PC runs ubuntu: on the Companion Computer / PC, go to network settings > Wired > settings. Go to IPv4 settings and add your IPADDR of your choising (ex: 192.168.0.6) and the NETMASK 255.255.255.0
6.If the companion computer runs rapsbian (e.g. using CM4 baseboard), refer to this [guide](https://www.ionos.com/digitalguide/server/configuration/provide-raspberry-pi-with-a-static-ip-address/) to setup a static IP for your raspberry bi.
7. Try to ping the pixhawk ip from the companion computer by running the command `ping 192.168.0.4` from the terminal.
8. Go to params in QGC and change the following settings `MAV_x_CONFIG`, where `x` is the instance number. Our default configuration uses `x=2` instance for ethernet communication.
9.  In QGroundControl, search for the `MAV_x_CONFIG` parameter.
10. Set the value of the `MAV_x_CONFIG` parameter to `ethernet`.
11. Reboot the vehicle and relaunch QGroundControl.
12. Set the value of the `MAV_x_MODE` parameter to `Custom`.
13. Set the value of the `MAV_x_RATE` parameter to `2000000` B/s.
14. Set the value of the `MAV_x_BROADCAST` parameter to `1`.
15. Set the value of the `MAV_x_RADIO_CTL` parameter to `0`.
16. Set the value of the `MAV_x_REMOTE_PRT` parameter to `14540`.
17. Set the value of the `MAV_x_UDP_PRT` parameter to `14540`.
18. Reboot the vehicle.

For using MAVROS on the companion computer to communicate with the pixhawk over ethernet, you can run the following command:
```
roslaunch mavros px4.launch fcu_url:=udp://@IPADDR:14540@
```
where IPADDR is the ip address of the pixhawk (by default 192.168.0.4).


##### 2. Using UART

1. Choose the MAVLink instance to work on `MAV_x_CONFIG`, where `x` is the instance number. Our default configuration uses `x=1` instance for UART communication.
2. In QGroundControl, search for the `MAV_x_CONFIG` parameter.
3. Set the value of the `MAV_x_CONFIG` parameter to the port you are using (e.g., `TELEM 2`).
4. Reboot the vehicle and relaunch QGroundControl.
5. Set the value of the `MAV_x_MODE` parameter to `Custom`.
6. Set the value of the `MAV_x_RATE` parameter to `2000000` B/s.
7. Reboot the vehicle.

After completing these steps, your MAVLink instance should be configured and ready to use.


#### Disablig landing detector
The landing detector of the PX4 can disarm the drone if not configured properly. As such, it is recommended to disable this module by setting the below parameters in QGroundControl:
1. Set the value of the `COM_DISARM_LAND` parameter to `-1`.
2. Set the value of the `MPC_THR_HOVER` parameter to a low value (e.g. `0.25`).

#### Change Frequency for navigation topics
We are interested in 3 topics for now (`VEHICLE_ANGULAR_VELOCITY`, `VEHICLE_ATTITUDE`, and `VEHICLE_LOCAL_POSITION`).

###### 1. Change the Publishing Rate in the Mavlink
First, run `uorb top -1` in the Mavlink shell to check the publishing rate of each topic. Each topic of the three mentioned above should be **at least** 200 Hz.

To change the rate of `VEHICLE_ATTITUDE` and `VEHICLE_LOCAL_POSITION`:

1. Set the parameter `IMU_INTEG_RATE` to be 400 Hz.
2. Set the parameter `EKF2_PREDICT_US` to be 2500.

Note: The parameter `IMU_INTEG_RATE` is to change the rate of the `VEHICLE_ATTITUDE` topic.

Note: The `EKF2_PREDICT_US` is to change the rate of the `VEHICLE_LOCAL_POSITION` topic, however, the topic rate will be limited to the `IMU_INTEG_RATE` parameter value.

Run `uorb top -1` in the Mavlink shell to re-check the publishing rate of the topics after applying the changes.


###### 2. Change the Stream Rate in MAVLINK_MODE_CUSTOM

1. Open `~/PX4-Autopilot/src/modules/mavlink/mavlink_main.cpp`

2. Change the three lines of the topics to be:
```
configure_stream_local("VEHICLE_LOCAL_POSITION", 200.0f);
configure_stream_local("VEHICLE_ATTITUDE", 200.0f);
configure_stream_local("VEHICLE_ANGULAR_VELOCITY", 200.0f);
```
3. Save, build, and upload to the pixhawk 6x. To [build](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html#building-firmware) PX4 for pixhawk 6c:

```
cd ~/PX4-Autopilot/
make px4_fmu-v6x_default upload
```
A successful run will end with this output:
```
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

###### 3. Check the Topics Frequency in Mavros
1. Close QGroundControl if open.
2. Unplug and re-plug the pixhawk.
3. run:
```
roslaunch mavros px4.launch
```
4. Then run each topic in a separate terminal:
```
rostopic hz /mavros/vehicle_angular_velocity/vehicle_angular_velocity
rostopic hz /mavros/vehicle_attitude/vehicle_attitude
rostopic hz /mavros/vehicle_local_position/vehicle_local_position
```
The output should be like this:
```
average rate: 200.00
        min: 0.002s max: 0.019s std dev: 0.00572s window: 100
```
If the rate is successfully 200 Hz, please start from step 2 again but with 400.0f Hz.


# How To Use in SITL
### Run in the terminal:
```
cd ~/PX4-Autopilot/
make px4_sitl gazebo
```

### Initialize mavros in another terminal:
```
roslaunch mavros px4.launch fcu_url:=udp://:14550@14557
```
### Now run the `offb` node in another terminal:
```
rosrun offb offb_node
```

