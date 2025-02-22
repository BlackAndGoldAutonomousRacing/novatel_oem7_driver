# NovAtel OEM7 Driver
[**ROS**](https://www.ros.org) Driver for [**NovAtel**](https://www.novatel.com) OEM7 GNSS/SPAN Receivers.  

## Getting Started
This documents how to custom-build the novatel_oem7_driver for ROS from the provided source code. Typical users will prefer to 
install the pre-made binary release that has been published in the ROS distribution.

Refer to our documentation under the ROS community wiki for:
 * Hardware Setup
 * Binary Driver Installation
 * Driver Configuration
 * Driver Runtime Operation
 * Post-Processing data
 * Information on Relevant NovAtel Services and Products
 * Advanced Topics

novatel_oem7_driver documentation on ROS community wiki is located here:
http://wiki.ros.org/novatel_oem7_driver

<HR>

## Building novatel_oem7_driver from source code
### Prerequisites
* Install ROS2 Humble.
* Obtain OEM7 receiver.


### Installation
#### Option A: Install binary package (not supported yet; coming soon)
There is substantial documention regarding use of the binary release of this driver on the ROS community wiki, located here:
https://wiki.ros.org/novatel_oem7_driver

The key step is:
```
sudo apt install ros-${ROS_DISTRO}-novatel-oem7-driver
```

Please refer to the Community Wiki for detailed run-time documentation for novatel_oem7_driver (link given above).


#### Option B: Build from source (docker)
These instructions assume that you are using Ubuntu 22.04.

1. Install Docker, add the user you intend on using to the 'docker' group. For example:
   1. Add the current user to the 'docker' group: `sudo usermod -aG docker ${USER}`
   1. Apply the membership changes to the current session: `su - ${USER}`
1. From the base directory of the repository, create container for the desired ROS architecture and distro, e.g. Humble:  
   `./docker/run.sh -r amd64 humble`  
   Note: only amd64 and arm64v8 architectures are supported at this point.  
1. From within your docker container, use standard ROS2 tools, like colcon.
1. Alternatively, use the build.sh script.

#### Option C: Build from source (local environment)
Here are approximate instructions for building this driver with your local ROS development environment. Please note this is for reference. The Docker approach is recommended.

1. Install ROS with developer support to your environment ([**ROS Wiki Ubuntu**](http://wiki.ros.org/Installation/Ubuntu))
1. Install ROS dependencies using `rosdep install --from-paths src --ignore-src -r -y`
1. Run `source /opt/ros/${ROS_DISTRO}/setup.bash`
1. Run build: `./build.sh -f`
1. After a successful build, source your local environment: . install/setup.sh

#### Install .deb packages 
Building produces two deb package, novatel-oem7-driver and novatel-oem7-msgs.

You can then install these via `apt` or `dpkg`:
```
sudo apt install ./ros-{$ROS_DISTRO}-novatel-oem7*.deb
```

## Configuration Parameters
This driver assumes `SETINSROTATION RBV 0 0 0 * * *`, `SETINSROTATION USER 0 0 0 * * *` and `SETINSTRANSLATION USER 0 0 0 * * *`. The driver is tested with a NovATel PwrPak 7D unit. The internal frames are assumed to be (w.r.t. the receiver unit enclosure):
- RAWIMU: X - Right, Y - Front, Z - Up (as specified by the INS origin marker on the enclosure.)
- INSPVA: X - Left, Y - Front, Z - Down. Orientation origin: Upright, X facing North.
- CORRIMU: X - Left, Y - Front, Z - Up

The driver outputs all processed ROS messages in the following frame:
- ROS: X - Forward, Y - Left, Z - Up. Orientation origin: Upright, X facing East.

The ROS frame is equivalent to rotating RAWIMU by +90 degrees along its Z axis.

Parameters:
* **oem7_position_source**: string.
  Oem7 log used to obtain the position for GPSFix, NavSatFix, Odometry.

  * **BESTPOS**: Always use BESTPOS log

  * **INSPVAS**: Always use INSPVAS log. Requires SPAN

  * unset (default): use INSPVAS unless BESTPOS has better quality

* **oem7_imu_rate**: int.
  Overrides the IMU data rate. Can be used to enable IMU message generation when INSCONFIG is not available, e.g. when feeding input from a partial dataset file.

  * 0 (default): INSCONFIG must be present in input.

* **oem7_odometry_zero_origin**: bool
  When `true`, use `0,0,0` as Odometry origin; the first valid GPSFix is used to set the origin.

  - false (default)

* **oem7_odometry_transform**: bool.
  When `true`, Transform is published, sourced from Odometry. 

  * false (default)

* **oem7_receiver_log_file**: string.
  Path to a file recording all bytes output by the Oem7 Receiver.

  * "" (default): no output is captured.

* **oem7_decoder_log_file**: string.
  Path to a file recording all bytes output by the Oem7 Decoder. Used for decoder debugging.

  * "" (default): no output is captured.

* **oem7_strict_receiver_init**:bool.
  When `true`, strict initialization is used; no position / velocity /attitude data is output unless all receiver initialization commands have succeeded.

  * true (default)

* **oem7_publish_unknown_oem7_raw**: bool.
  When `true`, all Oem7 messages not supported by the decoder are publshed as `raw`.

  * false (default)

* **oem7_publish_delay**: double.
  Seconds to delay before publishing each message. Used for debugging, and with file-based input.

  * 0.0 (default)

## Next Steps
Refer to the novatel_oem7_driver documentation in the ROS wiki for more information:
http://wiki.ros.org/novatel_oem7_driver


## Authors

* [**NovAtel**](https://www.novatel.com), part of [**Hexagon**](https://hexagon.com)
* Haoguang Yang, adaptation of the driver for Indy Autonomous Challenge.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


