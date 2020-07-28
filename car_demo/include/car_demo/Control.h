#pragma once
#include <iostream>
using namespace std;

#include <car_demo/Controller/PIDController.h>
#include <car_demo/Controller/ControllerAgent.h>
#include <car_demo/TrajectoryAnalyzer.h>
#include <car_demo/VehicleState.h>
#include <car_demo/Common.h>
#include <car_demo/Controller/Controller.h>
#include <car_demo/Controller/DoublePID.h>
#include <car_demo/Controller/LQRController.h>
#include <car_demo/Controller/MPCController.h>
#include <car_demo/Controller/PurePursuit.h>

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
#include <prius_msgs/VehicleInfo.h>


const string CONTROL_CONF_FILE_NAME = "/home/ht/ControlModule/src/car_demo/conf/control_conf.txt";

class Control
{
    public:
    Control()
    {
        ROS_INFO("Control Node had been created!");
    }
    Status Init();
    Status Start();
    void Stop();
    int Spin();
    Status ReadControlConf(string control_conf_file_name, ControlConf &control_conf);
    void TrajectoryCallback(const prius_msgs::My_Trajectory &trajectory);
    void LocalizationCallback(const nav_msgs::Odometry &localization);
    void VehicleStateCallback(const prius_msgs::VehicleInfo &current_state);
    void WriteInDebug(ofstream &ofs, prius_msgs::Augmented_My_Trajectory_Point vehicle_info);


    private:
    double init_time = 0.0;
    Status status;
    ControlConf control_conf;
    ControllerAgent controller_agent;
    TrajectoryAnalyzer trajectory_analyzer;
    VehicleState vehicle_state;
    prius_msgs::Control control_cmd;
    ofstream ofs;
    string simulation_result="/home/ht/ControlModule/src/car_demo/plot_py/simulation_result.txt";


    ros::NodeHandle node;
    ros::Publisher command_pub;
    ros::Publisher vehicle_info_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber localization_sub;
    ros::Subscriber vehicle_state_sub;
};