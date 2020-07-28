#pragma once
#include <iostream>
using namespace std;

#include <car_demo/TrajectoryAnalyzer.h>
#include <car_demo/VehicleState.h>
#include <car_demo/Common.h>
#include <car_demo/Controller/Controller.h>
#include <car_demo/Controller/DoublePID.h>
#include <car_demo/Controller/PurePursuit.h>
#include <car_demo/Controller/MPCController.h>
#include <car_demo/Controller/LQRController.h>
#include<car_demo/Controller/StanleyController.h>
#include<car_demo/Controller/RearWheelFeedbackController.h>

#include <algorithm>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <math.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>
#include <prius_msgs/Augmented_My_Trajectory_Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <fstream>




class ControllerAgent
{
    public:
    ControllerAgent()
    {
        ROS_INFO("Controller Agent is created!");
    }
    Status Init(ControlConf &control_conf);
    Status ComputeControlCmd( TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    Status RegisterController(ControlConf &control_conf);
    void CurrentController();

    vector<Controller*> controller_list;
    int controller_num;
    string longitudinal_controller;
    string lateral_controller;
    string centralized_controller;

    private:
    Status status;
    double Ts;


};