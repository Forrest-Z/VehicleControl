# Table of Contents
* [Introduction](#introduction)
* [WorkFlow](#workflow)
* [Usage](#usage)
    * [Run Example](#run-example)
    * [Choose Test Trajectory](#choose-test-trajectory)
    * [Record Trajectory](#record-trajectory)
    * [Choose Controller and Adjust Their Parameters](#choose-controller-and-adjust-their-parameters)
* [Controllers](#controllers)
    * [Controller list](#controllers)
    * [Result PLot Reverse](#result-pLot-reverse)
    * [Result PLot Forward](#result-pLot-forward)
    * [Other Plot](#other-plot)
* [Maintainers](#maintainers)

# Introduction
This repository includes the commonly used control algorithms for vehicle motion like **PID, LQR, MPC** 
and other algorithms based on geometry model like **Stanley, Pure Pursuit** and **Rear Wheel Feedback**.  

**Two scenarios** are tested here:
* forward driving in curved road with differernt velocity and radius
* reverse driving in curved road

The velocity of test trajectory is in the range from 10m/s to 25m/s, a part of result plot is post in below, all the plots can be found in the folder named **result_plot**.

The simulation is conducted in GAZEBO, the vehicle model and the senario are obtained from the Demo of Prius in ROS/GAZEBO(https://github.com/osrf/car_demo.git). 

![Prius_image](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/Prius_image.png)
![Prius_image_2](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/Prius_image_1.jpg)
![senario]![senario](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/senario_map.png)


# WorkFlow
![work_flow](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/workflow.png)

# Usage
## Run Example
```
~$ cd catkin_ws/src//car_demo/multi_launch/
# forward running in an empty world with smooth, artificial trjaectory
~/catkin_ws/src//car_demo/multi_launch$ ./forward.sh
# reverse running in an empty world with smooth, artificial trjaectory
~/catkin_ws/src//car_demo/multi_launch$ ./reverse.sh
# running in an simulated environment with recored trajectory
~/catkin_ws/src//car_demo/multi_launch$ ./simulated_world.sh
# running in an simulated environment with ramp
~/catkin_ws/src//car_demo/multi_launch$ ./run_in_ramp.sh
```  
![working process](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/working_process.gif)

## Choose Test Trajectory
```
~$ cd catkin_ws/src/car_demo/launch/
# change the trajectory in empty world
~/catkin_ws/src//car_demo/launch$ gedit publish_mat_trajectory.launch
# change the trajectory in simulated world
~/catkin_ws/src//car_demo/launch$ gedit publish_recorded_trajectory.launch
```
Modify the value of test trajectory file path to change the test trajectories.The availble test trajectories can be found in the folder named **"test_trajectories"**.  

In this folder, the trajectory file starting with "mat" means that it is a artificial trajectory, and the recorded trajectories are recorded in the GAZEBO senario, which are not so smooth, but closer to the real situation. 

The last num of trajectory file name means the velocity of vehicle running in curve, **only for the "mat trajectories"**, the velocity of recorded trajectories are not constant.

## Record Trajectory
Recording a trajectory is a good method to generate an availble trajectory in the city environment.  

1. find the plugin code  
```
~$ cd catkin_ws/src/car_demo/plugins/
~/catkin_ws/src//car_demo/plugins$ gedit PriusHybridPlugin.cc
``` 

2. Switch on the recording function  
correct the **record_enabled** into **True**  

3. roslaunch 
```
~$ roslaunch car_demo record_trajectory.launch
``` 

4. record trajectory  
"W","A","D" are the control keys  
"S" is "reverse"  
"E" is "brake"

5. Get recorded trajectory  
The recorded trajectory is stored in the folder (catkin_ws/src/car_demo/recorded_trajectories), you can trim it and move it to the **test_trajectories** folder.


## Choose Controller and Adjust Their Parameters
```
~$ cd catkin_ws/src/car_demo/conf/
~/catkin_ws/src//car_demo/conf$ gedit control_conf.txt
```
This file includes the physical parameters of vehicle model (can not be modified) and all the parameters (can be adjusted) required by the controllers and other important objects necessary during the operation of control module.  

The controller can be classified into the distributed controllers (control longitudinal and lateral motion respectively) and centralized controllers (control the vehicle motion coupled).  

To active distributed controller, the **CONTROLLER_NUM** must be 2, the next step is to set the **LONGITUDINAL_CONTROLLER** and **LATERAL_CONTROLLER**, the options are listed below them. 

To active centralized controller, the **CONTROLLER_NUM** must be 1, the next step is to set the **CENTRALIZED_CONTROLLER**.  

## Choose the Plot shown
Since the FPS is low if we plot more than one figures simultaneously, so now there is only one plot can be shown during the simulation (after simulation, we can get all the plots, see [Other Plot](#other-plot)), the steps to choose the plot shown is:
```
~$ cd catkin_ws/src/car_demo/scripts/
~/catkin_ws/src//car_demo/scripts$ gedit TrajectoryPlotter.py
``` 
change the **TARGET_PLOT** to choose which plot you want to see during simulation, now there are three options
* trajectory comparison
* velocity comparison with station
* heading angle comarison with station


# Controllers
* Longitudinal Controllers
    * [PID controller](#result_plot)
* Lateral Controllers
    * [Pure Pursuit Controller](#pure-pursuit-controller)
    * [Stanley Controller](#stanley-ontroller)
    * [Rear Wheel Feedback Controller](#rear-wheel-feedback-controller)
    * [LQR](#lqr)
    * [LQR with prediction](#lqr-with-prediction)
    * [MPC](#mpc)
* Centralized Controllers
    * MPC (working in process)
## Result PLot Reverse
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/pure_pursuit_reverse_10.gif) 
## Result PLot Forward
### Pure Pursuit Controller  
* Trajectory Tracking
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/pure_pursuit_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/pure_pursuit_10.gif) 
* Velocity and Heading Tracking(working in process)  
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Pure_Pursuit_10_v.gif)
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Pure_Pursuit_10_yaw.gif)
### Stanley Controller  
* Trajectory Tracking
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Stanley_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Stanley_15.gif)  
* Velocity and Heading Tracking(working in process)    
### Rear Wheel Feedback Controller  
* Trajectory Tracking   
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Rear_Wheel_Feedback_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/Rear_Wheel_Feedback_15.gif)  
* Velocity and Heading Tracking(working in process)    
### LQR  
* Trajectory Tracking  
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/LQR_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/LQR_15.gif)  
* Velocity and Heading Tracking(working in process)    
### LQR with prediction  
* Trajectory Tracking   
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/LQR_prediction_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/LQR_prediction_15.gif)  
* Velocity and Heading Tracking(working in process)    
### MPC  
* Trajectory Tracking   
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/MPC_10.gif)
![v=15m/s](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/gif/MPC_15.gif)  
* Velocity and Heading Tracking(working in process)  


## Other Plot
```
~$ cd catkin_ws/src/car_demo/plot_py/
~/catkin_ws/src/car_demo/plot_py$ python result_plot.py
```
This program subscribes the published trajectory, vehicle localization and the vehicle working conditions like output torque, brake torque, traveled distance and so on. 

It is very convenient to get these plots through simple modification of this code.

# Maintainers
[@djh1995555](https://github.com/djh1995555/)




