#!/bin/bash 

roslaunch car_demo run_in_ramp.launch &
echo "launch world successfully"
sleep 1

roslaunch car_demo trajectory_plot.launch &
echo "launch trajectory_plot successfully"

sleep 10

roslaunch car_demo my_publisher.launch &
echo "publish trajectory successfully"


wait
exit 0
