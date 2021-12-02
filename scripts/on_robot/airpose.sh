#!/bin/bash

roslaunch airpose_client one_robot.launch robotID:=$MACHINE host:=192.168.9.3 port:=9900

echo "AIRPOSE failed with errorcode $?" >>${LOGDIR}/current/faillog
