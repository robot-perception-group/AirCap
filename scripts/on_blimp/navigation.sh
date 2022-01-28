#!/bin/bash

roslaunch blimp_nmpc_wrapper_node one_blimp.launch robotID:=$MACHINE NUM_ROBOTS:=$CCOUNT

echo "nmpc_planner failed with errorcode $?" >>${LOGDIR}/current/faillog
