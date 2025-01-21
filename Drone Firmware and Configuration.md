# Drone Firmware and Configuration

## Installation of Firmware

Pre-built firmware is available in this repository under `/Systems/PX4/Firmware&Parameters/Firmware`. The firmware may vary depending on the flight controller hardware used in your drone. Currently, only firmware for the Pixhawk 6X is available, and firmware for the Kakute H7 will be added soon.

Upload the firmware to the flight controller through QGroundControl. The firmware has the extension `.px4`

If you require firmware for other Pixhawk versions, please contact `Abdulla Ayyad` at `abdullah.ayyad@droneleaf.io`.

## Setting Parameters & Flight Configuration

You need to manually set the `SYS_AUTOSTART` parameter on PX4 throug QGroundControl to one of the below supported values based on your configuration. Please note that you need to reboot the pixhawk for changes to take effect.


| Value | Drone Class | Communication with Companion Computer | Navigation             |
| ------- | ------------- | --------------------------------------- | ------------------------ |
| 4951  | Quadrotor   | Ethernet                              | Motion Capture         |
| 4952  | Quadrotor   | Serial (Telem2)*                      | Motion Capture         |
| 4953  | Octarotor   | Ethernet                              | Motion Capture         |
| 4954  | Octarotor   | Serial (Telem2)*                      | Motion Capture         |
| 4955  | Quadrotor   | Ethernet                              | GPS + Baro + MAG       |
| 4956  | Quadrotor   | Ethernet                              | UM982 RTK + Baro + MAG |

* Serial communication on Telem 2 port is assumed with a baud rate of 921600. For any other baud rate, you can set the `SER_TEL2_BAUD` parameter manually.

For further custom configurations, refer to [this link](https://docs.px4.io/v1.12/en/dev_airframes/adding_a_new_frame.html) on how to create custom airframe configurations.

For support, please contact `Abdulla Ayyad` at `abdullah.ayyad@droneleaf.io`.

## Known issues

1) Motor configurations missing from 4955.
2) Message "QGroundControl was not able to retreive the full set of parameters ..." appears due to sleep at boot-up.
3) 

# Advanced: Build your own airframe
** Note **: Folder paths are relative in the PX4-Autopilot directory.

1. Setup your PX4 development environment by following the instructions in: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
2. Clone from https://github.com/DroneLeaf/PX4-Autopilot/ branch HEAR_v2
3. Add your airframe to `ROMFS/px4fmu_common/init.d/airframes/`:
    1. First add the new airframe. Take for example 4951 airframe.
    2. Add the firmware to the CMakeLists.txt in the same directory.
4. Compile the firmware for the intended hardware, e.g. `make px4_fmu-v6x_default`
5. In the build folder you will find the new firmware, e.g. `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
6. Upload the firmware through QGC to the board.
7. From parameters change `SYS_AUTOSTART` parameters to match your new airframe. 
8. Reboot and that is it!

