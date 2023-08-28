### These instructions are used for Ubuntu 20.04. They could be used in WSL2 or as a dual boot. But it's highly recommended to be used as a dual boot.

# ROS Noetic
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

# Mavros
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

# PX4 Firmware
### Deps Before Cloning:
These installations solved all of the errors I faced.
```
sudo apt install python3-pip -y
pip3 install kconfiglib
sudo apt install gcc-arm-none-eabi -y
pip3 install --user jinja2
pip3 install --user jsonschema
```

### Cloning PX4:
Clone the branch `MK` from `Mu99-M/PX4-Autopilot` in home directory.
```
cd ~
git clone -b MK https://github.com/Mu99-M/PX4-Autopilot.git --recursive
```
### Install gstreamer for SITL:
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```
### Update (common.xml) & (iris.sdf.jinja):
Unfortunately, I wasn't able to push these 2 files for some reason, so they have to be updated manually. First, build px4. It will show an error and it won't continue, but it's ok for now.

```
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

1. common.xml

Download the file from DroneLeaf onedrive [here](https://droneleaf.sharepoint.com/:u:/s/technical/ET_WUqM3u3xOqpPJwsCv1UwBCr1k-d1219Qgi8GG4Kp_vg?e=UNfQKu).

Then, copy the downloaded file to this path `~/PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/`. Or run the following command in the downloaded file path.
```
cp ./common.xml ~/PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/
```
2. iris.sdf.jinja

Open the file `~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`. Search for `</enable_lockstep>` and change its value from `1` to `0`.

### Building PX4:
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

# offb Node
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
Now everthing is ready to be used.

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
# Configuring the MAVLink Instance

1. Connect the pixhawk to QGC using a USB-C cable.
2. Open QGroundcontrol > Analyze Tools > MAVLink Console.
3. Enter the following commands:        echo DEVICE=eth0 > /fs/microsd/net.cfg
                                        echo BOOTPROTO=static >> /fs/microsd/net.cfg
                                        echo IPADDR=10.0.0.4 >> /fs/microsd/net.cfg
                                        echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg
                                        echo ROUTER=10.0.0.1 >>/fs/microsd/net.cfg
                                        echo DNS=10.0.0.1 >>/fs/microsd/net.cfg
   The IPADDR could be chosen by the user, The router and the DNS addresses are depending on the router configurations and can be obtained under IPv4 tab in the Wi-Fi settings of the network.
4. Connect the pixhawk to an external power source, and connect the ethernet cable to the pc.
5. Go to network settings > Wired > settings.
6. Go to IPv4 settings and add your IPADDR of your choising (ex: 10.0.0.6) and the NETMASK 255.255.255.0
7. Try to ping the pixhawk ip from the terminal to ensure the connection is established.
8. Choose the MAVLink instance to work on `MAV_x_CONFIG`, where `x` is the instance number.
9. In QGroundControl, search for the `MAV_x_CONFIG` parameter.
10. Set the value of the `MAV_x_CONFIG` parameter to the port you are using (e.g., `TELEM 1`).
11. Reboot the vehicle and relaunch QGroundControl.
12. Set the value of the `MAV_x_MODE` parameter to `Custom`.
13. Set the value of the `MAV_x_RATE` parameter to `2000000` B/s.
14. Reboot the vehicle.

After completing these steps, your MAVLink instance should be configured and ready to use.

# Change Frequency for a Topic
We are interested in 3 topics for now (`VEHICLE_ANGULAR_VELOCITY`, `VEHICLE_ATTITUDE`, and `VEHICLE_LOCAL_POSITION`).

## 1. Change the Publishing Rate in the Mavlink
First, run `uorb top -1` in the Mavlink shell to check the publishing rate of each topic. Each topic of the three mentioned above should be **at least** 200 Hz.

To change the rate of `VEHICLE_ATTITUDE` and `VEHICLE_LOCAL_POSITION`:

1. Set the parameter `IMU_INTEG_RATE` to be 400 Hz.
2. Set the parameter `EKF2_PREDICT_US` to be 2500.

Note: The parameter `IMU_INTEG_RATE` is to change the rate of the `VEHICLE_ATTITUDE` topic.

Note: The `EKF2_PREDICT_US` is to change the rate of the `VEHICLE_LOCAL_POSITION` topic, however, the topic rate will be limited to the `IMU_INTEG_RATE` parameter value.

Run `uorb top -1` in the Mavlink shell to re-check the publishing rate of the topics after applying the changes.

## 2. Change the Stream Rate in MAVLINK_MODE_CUSTOM

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
## 3. Check the Topics Frequency in Mavros
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
