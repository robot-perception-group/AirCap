#!/bin/bash

#roslaunch camera_configs publish_info_robot.launch robotID:=$MACHINE
roslaunch camera_configs capture_logitech_c920.launch robotID:=$MACHINE

echo "camera capture and calibration publisher failed with errorcode $?" >>${LOGDIR}/current/faillog
