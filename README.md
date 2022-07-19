# Swift Nav driver for ROS2

## Contents

- [Overview](#overview)
- [Scope](#scope)
- [Dependencies](#dependencies)
- [Building](#building)
- [Params](#params)
- [Settings](#settings-in-piksi-console)
- [Run](#run)
- [Topics](#topics)
- [Further reading](#further-reading)
## Overview

This is a C++ ROS driver for Swift Nav Piksi Multi GPS / GNSS Receivers. The code is based on offical Swiftnav libswiftnav C example.

## Scope
The current version supports (not tested elsewhere):
- Ubuntu 20.04
- ROS2 Foxy
- Colcon tools
- Communication over serial

## Dependencies
### Libsbp
* libsbp (Swift binary protocol library) C client library from GitHub: https://github.com/swift-nav/libsbp
* Install libserialport: https://github.com/sigrokproject/libserialport

### Installing dependencies with script
```
cd sdk
sudo ./sdk-install
```

## Building
```
colcon build --packages-select swift_nav_driver
```

## Params
In `param/defaults.param.yaml` file:
* baudrate - driver was tested with 230400, default value set on Piksi is 115200
* port - must be set port name found in /dev for the device
* frame_id - frame id for the device, default value: "gnss"

## Settings in Piksi console
In Swift Console software:
* set imu rate
* enable and login to ntrip provider
* in used uart settings make sure that needed messages are enabled
  * msg_pos_llh_cov_t - 0x0211 - 529
  * msg_vel_ned_cov_t - 0x0212 - 530
  * msg_gps_time_t - 0x0102 - 258
  * msg_imu_raw_t - 0x0900 - 2304/20 (number after / divides rate of set imu rate by int number)
  

## Run
```
ros2 launch swift_nav_driver swift_nav_driver.launch.py
```

Alternatively you can use a ros2 run
```
ros2 run swift_nav_driver swift_nav_node_exe
```


## Topics
Driver publishes the following topics and [types]:
* `/gnss/nav_fix` Type: `sensor_msgs/msg/NavSatFix`
* `/gnss/nav_speed` Type: `geometry_msgs/msg/TwistWithCovarianceStamped`
* `/gnss/nav_time` Type: `sensor_msgs/msg/TimeReference`
* `/gnss/imu` Type: `sensor_msgs/msg/Imu`




## Further reading
- Libswiftnav: https://github.com/swift-nav/libswiftnav
- Libsbp documentation: https://swift-nav.github.io/libsbp/c/build/docs/html

