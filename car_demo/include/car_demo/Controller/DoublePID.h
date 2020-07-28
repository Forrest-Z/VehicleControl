#pragma once
#include <iostream>
using namespace std;
#include <car_demo/Common.h>
#include <car_demo/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <car_demo/Controller/PIDController.h>

class DoublePID:public Controller
{
    public:
    Status Init(ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd);


    PIDController velocity_PID;
    PIDController acceleration_PID;
 

    double acc_compensation;
    double velocity_compensation;
    double slope_compensation;

    double torque_cmd;

    double Ts;
    double velocity_kp;
    double velocity_ki;
    double velocity_kd;
    double acc_kp;
    double acc_ki;
    double acc_kd;

};


