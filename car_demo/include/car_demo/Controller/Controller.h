#pragma once
#include <iostream>
using namespace std;
#include <car_demo/Common.h>
#include <car_demo/TrajectoryAnalyzer.h>
#include <car_demo/VehicleState.h>
#include <prius_msgs/Control.h>




class Controller
{
    public:
    virtual Status Init(ControlConf &control_conf)=0;
    virtual Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd)=0;
    string GetName();
    Status Reset();
    void Stop();

    string name;
    Status status;
};
