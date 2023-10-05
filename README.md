# IRT NTRIP Client and Server Caster
This is the official package of ntrip utilities including two basic nodes:
1. ntrip_ros_node: written in c++, given ntrip server configurations:
  	1. subscribing rover position in sensor_msgs::NavSatFix to update the GGA reference
  	2. parsing and publishing RTCM1004, RTCM1005, RTCMv3 and RTCML1E1 messages
  	3. (optinal) sending RTCM messages using Flatbuffers via UDP (for some real-time controllers with no ROS support)
  	3. providing ROS service to update ntrip server configurations
  	
2. reference_station_tracker: written in python, given rover position, automatically change the pre-defined ntrip server configurations (ip, reference station, user information) for
	1. ntrip_ros_node
	2. GNSS receivers (currently only NovAtel OEM7 and ublox via ublox ros)
	
### Currently, both ROS2 and ROS1 are supported. The ROS1 version has not been validated for long-term operations.

---

## Dependencies:

1. [ros2](https://docs.ros.org/en/rolling/Installation.html)
2. [irt_nav_common](https://github.com/rwth-irt/irt_nav_common)
3. [novatel_oem7_msgs](https://github.com/rwth-irt/novatel_oem7_driver)
4. [geopy](https://github.com/geopy/geopy)
5. [paho mqtt](https://github.com/eclipse/paho.mqtt.python)
6. [flatbuffers](https://flatbuffers.dev/)


---

## Suported GNSS Receivers:
1. NovAtel OEM7 (e.g., pwrpak7 or span)
2. Ublox F9P 

---

## ToDos:
1. set optinal functionalities (e.g., flatbuffer) as optional if flautbuffer is not installed

## Maintance:
1. [Haoming Zhang](mailto:h.zhang@irt.rwth-aachen.de)