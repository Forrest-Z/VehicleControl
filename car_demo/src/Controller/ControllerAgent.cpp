#include <car_demo/Controller/ControllerAgent.h>


Status ControllerAgent::Init(ControlConf &control_conf)
{
    Ts = control_conf.conf_param.Ts;
    Status reg_status = RegisterController(control_conf);
    if(reg_status.status != "OK")
    {
        ROS_INFO("Controller registration failed!");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Controller registration successfully!");
    CurrentController();
    switch(controller_num)
    {
        case 2:
            if(longitudinal_controller == "Double PID Controller")
            {
                Controller* double_pid_ptr = new DoublePID;
                controller_list.push_back(double_pid_ptr);
                ROS_INFO_STREAM(longitudinal_controller<<" is created successfully!");
            }

            if(lateral_controller == "Pure Pursuit Controller")
            {
                Controller* pure_pursuit_ptr = new PurePursuit;
                controller_list.push_back(pure_pursuit_ptr);
                ROS_INFO_STREAM(lateral_controller<<" is created successfully!");
            }
            else if(lateral_controller == "LQR Controller")
            {
                Controller* lqr_controller_ptr = new LQRController;
                controller_list.push_back(lqr_controller_ptr);
                ROS_INFO_STREAM(lateral_controller<<" is created successfully!");
            }
            else if(lateral_controller == "MPC Controller")
            {
                Controller* mpc_controller_ptr = new MPCController;
                controller_list.push_back(mpc_controller_ptr);
                ROS_INFO_STREAM(lateral_controller<<" is created successfully!");
            }
            else if(lateral_controller == "Rear Wheel Feedback Controller")
            {
                Controller* RWF_controller_ptr = new RearWheelFeedbackController;
                controller_list.push_back(RWF_controller_ptr);
                ROS_INFO_STREAM(lateral_controller<<" is created successfully!");
            }
            else if(lateral_controller == "Stanley Controller")
            {
                Controller* Stanley_controller_ptr = new StanleyController;
                controller_list.push_back(Stanley_controller_ptr);
                ROS_INFO_STREAM(lateral_controller<<" is created successfully!");
            }
            
            break;
        case 1:
            if(centralized_controller == "MPC Controller")
            {
                Controller* mpc_controller_ptr = new MPCController;
                controller_list.push_back(mpc_controller_ptr); 
                ROS_INFO_STREAM(centralized_controller<<" is created successfully!");              
            }
            break;
    }
    ROS_INFO("Controller list is created!");
    for(vector<Controller*>::iterator it = controller_list.begin();it!=controller_list.end();it++)
    {
        (*it)->Init(control_conf);
    }
    ROS_INFO("Controllers init successfully!");
    status.status = "OK";
    return status;
}

Status ControllerAgent::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Start compute command!");
    for(vector<Controller*>::iterator it = controller_list.begin();it!=controller_list.end();it++)
    {

        (*it)->ComputeControlCmd(trajectory_analyzer,vehicle_state,control_cmd);
    }

    status.status = "OK";
    return status;
}

Status ControllerAgent::RegisterController(ControlConf &control_conf)
{
    controller_num = control_conf.conf_param.controller_num;
    switch(controller_num)
    {
        case 2:
            longitudinal_controller = control_conf.LONGITUDINAL_CONTROLLER;
            lateral_controller = control_conf.LATERAL_CONTROLLER;
            break;
        case 1:
            centralized_controller = control_conf.CENTRALIZED_CONTROLLER;
            break;
    }
    status.status = "OK";
    return status;
}

void ControllerAgent::CurrentController()
{
    switch(controller_num)
    {
        case 2:
            ROS_INFO_STREAM("Current controllers are "<<longitudinal_controller<<"+"<<lateral_controller);
            break;
        case 1:
            ROS_INFO_STREAM("Current controller is "<<centralized_controller);
            break;  
    }
}