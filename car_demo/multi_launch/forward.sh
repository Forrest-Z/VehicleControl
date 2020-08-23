#!/bin/bash 

roslaunch car_demo run_in_empty_world.launch &
echo "launch world successfully"
sleep 1

roslaunch car_demo trajectory_plot.launch &
echo "launch trajectory_plot successfully"
sleep 1

roslaunch car_demo publish_ontime_trajectory.launch &
echo "publish local trajectory successfully"
sleep 10

roslaunch car_demo publish_mat_trajectory.launch &
echo "publish global trajectory successfully"

wait
exit 0
