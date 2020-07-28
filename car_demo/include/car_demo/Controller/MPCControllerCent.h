#pragma once
#include <iostream>
using namespace std;
#include <car_demo/Common.h>
#include <car_demo/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include "QuadProg++/Array.hh" 
#include "QuadProg++/QuadProg++.hh"
using namespace quadprogpp; 
using namespace Eigen;

class MPCController:public Controller
{
    public:
    Status Init(ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    void GetAugmentedMatrix(MatrixXd &matrix_augmented,int matrix_size,vector<double> v);
    void ComputeStateError(prius_msgs::My_Trajectory_Point &goal_state,VehicleState &vehicle_state);
    void CreatePredictionModel();

    

    int Nx=3; // 状态量个数
    int Nu=2; // 控制量个数
    double current_velocity=0.0;
    double planned_heading=0.0;

    double prediction_length=20;
    double control_length=20;
    double wheel_base;
    double previous_steering_angle=0.0;
    vector<double> V_Q;
    vector<double> V_R;
    double Ts;

    Matrix3d A=Matrix3d::Identity();
    MatrixXd B=MatrixXd::Zero(3,2);
    Matrix3d Q=Matrix3d::Identity();
    MatrixXd Q_augmented;
    MatrixXd R_augmented;

    MatrixXd A_augmented;
    MatrixXd B_augmented;


    Vector3d state_error = Vector3d::Zero();
    Vector2d control_output=Vector2d::Zero();


};