#!/bin/bash 

roslaunch car_demo run_with_carsim.launch &
echo "launch world successfully"
sleep 1

roslaunch car_demo trajectory_plot.launch &
echo "launch trajectory_plot successfully"

sleep 10

roslaunch car_demo publish_mat_trajectory.launch &
echo "publish trajectory successfully"


wait
exit 0
