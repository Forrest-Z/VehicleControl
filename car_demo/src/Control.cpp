#include<car_demo/Control.h>

int Control::Spin()
{
    Status init_status = Init();
    if(init_status.status != "OK")
    {
        ROS_INFO("Init Failed!");
        return -1;
    }

    Status start_status = Start();
    if(start_status.status != "OK")
    {
        ROS_INFO("Start Failed!");
        return -2;
    }

    ros::spin();
/*     ros::waitForShutdown();
    Stop();
    ROS_INFO_STREAM(name<<" exited!"); */
    return 0;
}
Status Control::Init()
{
    init_time = (ros::Time::now()).toSec();
    ROS_INFO("Control Init...");
    Status read_status = control_conf.ReadControlConf(CONTROL_CONF_FILE_NAME);

    int controllers_num;
    string longitudinal_controller;
    string lateral_controller;
    string centralized_controller;
    node.param<int>("controllers_num", controllers_num, 0);
    node.param<std::string>("longitudinal_controller", longitudinal_controller, "xx");
    node.param<std::string>("lateral_controller", lateral_controller, "xx");
    node.param<std::string>("centralized_controller", centralized_controller, "xx");
    node.setParam("controllers_num",int(control_conf.conf_param.controller_num));
    node.setParam("longitudinal_controller",control_conf.LONGITUDINAL_CONTROLLER);
    node.setParam("lateral_controller", control_conf.LATERAL_CONTROLLER);
    node.setParam("centralized_controller",control_conf.CENTRALIZED_CONTROLLER);
    if(read_status.status != "OK")
    {
        ROS_INFO_STREAM("Unable to load control conf file "<<CONTROL_CONF_FILE_NAME);
        status.status = "Failed";
        return status;
    }
    ROS_INFO_STREAM("Control conf file:"<<CONTROL_CONF_FILE_NAME<<" had beed loaded!");
    
    ofs.open(simulation_result,ios::out);
    if(!ofs.is_open())
    {
        ROS_INFO("打开文件失败！");
        status.status = "Failed";
        return status;
    }
    ROS_INFO("ready to write in!");

    Status vehicle_instance_status = vehicle_state.Init(control_conf);
    if(vehicle_instance_status.status != "OK")
    {
        ROS_INFO("Vehicle instance init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Vehicle instance init successfully!");

    Status analyzer_status = trajectory_analyzer.Init(control_conf);
    if(analyzer_status.status != "OK")
    {
        ROS_INFO("Trajectory analyzer init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }

    ROS_INFO("Trajectory analyzer init successfully!");
    Status agent_status = controller_agent.Init(control_conf);
    if(agent_status.status != "OK")
    {
        ROS_INFO("Controller agent init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }

    ROS_INFO("Controller agent init successfully!");
    status.status = "OK";
    return status;
}

Status Control::Start()
{
    ROS_INFO("COntrol resetting vehicle state, sleeping for 1000ms...");
    ros::Duration(1).sleep();
    vehicle_state.state = "WORKING!";
    ROS_INFO_STREAM("Vehicle current state is"<<vehicle_state.state);

    command_pub=node.advertise<prius_msgs::Control>("/prius", 10, true);
    vehicle_info_pub=node.advertise<prius_msgs::Augmented_My_Trajectory_Point>("/prius/vehicle_info", 10, true);
    trajectory_sub=node.subscribe("/prius/planning_output",10, &Control::TrajectoryCallback,this);
    localization_sub=node.subscribe("/base_pose_ground_truth",10,&Control::LocalizationCallback,this);
    vehicle_state_sub=node.subscribe("/prius/chassis_info",10,&Control::VehicleStateCallback,this);



    status.status = "OK";
    return status;
}
void Control::Stop()
{
    // TO DO
}



void Control::TrajectoryCallback(const prius_msgs::My_Trajectory &trajectory)
{
    Status trajectory_status =trajectory_analyzer.ReadTrajectory(trajectory);
    if(trajectory_status.status != "OK")
    {
        ROS_INFO("Read trajectory failed!");
        return;
    }
    trajectory_analyzer.PrintTrajectory();

}

void Control::LocalizationCallback(const nav_msgs::Odometry &localization)
{   
    
    Status localization_status=vehicle_state.GetVehicleStateFromLocalization(localization);
    if(localization_status.status != "OK")
    {
        ROS_INFO("Get vehicle state failed!");
        return;
    }
    ROS_INFO_STREAM("current x ="<<vehicle_state.movement_state.pose.position.x );
    ROS_INFO_STREAM("current y ="<<vehicle_state.movement_state.pose.position.y );
    vehicle_state.ComputeDistanceFromDestination(trajectory_analyzer.destination);
    ROS_INFO_STREAM("destination x ="<<trajectory_analyzer.destination.x);
    ROS_INFO_STREAM("destination y ="<<trajectory_analyzer.destination.y);
    vehicle_state.ComputeBrakeDistanceAhead();
    ROS_INFO_STREAM("Brake distance ahead is"<<vehicle_state.brake_distance_ahead);
    if (trajectory_analyzer.trajectory_info.empty()||vehicle_state.distance_from_destination<=vehicle_state.brake_distance_ahead)
    {
        // trajectory_analyzer.trajectory_info.clear();
        trajectory_analyzer.trajectory_info.clear();
        control_cmd.throttle = 0.0;
        control_cmd.brake = 1.0;
        if(trajectory_analyzer.trajectory_info.empty())
        {
            ROS_INFO("trajectory is empty!");
        }
        else
        {
            ROS_INFO("reach destination!");
        }
        ROS_INFO("Vehicle will stop!");
    }
    else
    {
        Status goal_status=trajectory_analyzer.MatchPointByPosition(vehicle_state);
        if(goal_status.status != "OK")
        {
            ROS_INFO("get the nearest point failed!");
            return;
        }

        ROS_INFO("Find the nearest point!");
        ROS_INFO_STREAM("goal x="<<trajectory_analyzer.goal_state.x );
        ROS_INFO_STREAM("goal y="<<trajectory_analyzer.goal_state.y );
        
        Status cmd_status=controller_agent.ComputeControlCmd(trajectory_analyzer,vehicle_state, control_cmd);
        if(cmd_status.status != "OK")
        {
            ROS_INFO("Compute control command failed!");
            trajectory_analyzer.trajectory_info.clear();
            control_cmd.throttle = 0.0;
            control_cmd.brake = 1.0;
            ROS_INFO("Vehicle will stop!");
            return;
        }
    }  
    command_pub.publish(control_cmd);
    //ROS_INFO("Control command is published!");
    vehicle_state.GetAllData(trajectory_analyzer.goal_state, trajectory_analyzer.preview_state,control_conf);
    vehicle_info_pub.publish(vehicle_state.vehicle_info);
    //ROS_INFO("Vehicle info is published!");
    WriteInDebug(ofs,vehicle_state.vehicle_info);
    //ROS_INFO("Write in debug successfully!");
    return;
}

void Control::VehicleStateCallback(const prius_msgs::VehicleInfo &current_state)
{
    Status chassis_status=vehicle_state.GetVehicleStateFromChassis(current_state);
    if(chassis_status.status != "OK")
    {
        ROS_INFO("Get vehicle state failed!");
        return;
    }
    ROS_INFO("Get vehicle state successfully!");
        
}
void Control::WriteInDebug(ofstream &ofs, prius_msgs::Augmented_My_Trajectory_Point vehicle_info)
{
    ofs<<"trajectory_point{"<<endl;
    ofs<<"  time: "<<vehicle_info.vehicle_info.header.stamp.toSec()<<endl;
    ofs<<"  x: "<<vehicle_info.trajectory_point.x<<endl;
    ofs<<"  y: "<<vehicle_info.trajectory_point.y<<endl;
    ofs<<"  velocity: "<<vehicle_info.trajectory_point.v<<endl;
    ofs<<"  acceleration: "<<vehicle_info.trajectory_point.a<<endl;
    ofs<<"  heading: "<<vehicle_info.trajectory_point.theta<<endl;
    ofs<<"  relative_time: "<<vehicle_info.trajectory_point.relative_time<<endl;
    ofs<<"  distance_error: "<<vehicle_info.distance_error<<endl;
    ofs<<"  velocity_error: "<<vehicle_info.velocity_error<<endl;
    ofs<<"  heading_error: "<<vehicle_info.heading_error<<endl;
    ofs<<"  goal_x: "<<vehicle_info.goal_x<<endl;
    ofs<<"  goal_y: "<<vehicle_info.goal_y<<endl;
    ofs<<"  preview_x: "<<vehicle_info.preview_x<<endl;
    ofs<<"  preview_y: "<<vehicle_info.preview_y<<endl;

    ofs<<"  x_from_chassis: "<<vehicle_info.vehicle_info.localization.x<<endl;
    ofs<<"  y_from_chassis: "<<vehicle_info.vehicle_info.localization.y<<endl;
    ofs<<"  z_from_chassis: "<<vehicle_info.vehicle_info.localization.z<<endl;
    ofs<<"  heading_from_chassis: "<<vehicle_info.vehicle_info.lateral_data.heading_angle<<endl;
    ofs<<"  vel_from_localization: "<<vehicle_info.vehicle_info.longitudinal_data.vel_from_localization<<endl;
    ofs<<"  vel_from_wheels: "<<vehicle_info.vehicle_info.longitudinal_data.vel_from_wheels<<endl;
    ofs<<"  acc_from_wheels: "<<vehicle_info.vehicle_info.longitudinal_data.acceleration<<endl;
    ofs<<"  travel_distance: "<<vehicle_info.vehicle_info.longitudinal_data.traveled_distance<<endl;
    ofs<<"  steering_wheel_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_angle_actual<<endl;
    ofs<<"  steering_wheel_expected: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_expected<<endl;
    ofs<<"  steering_wheel_error: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_error<<endl;
    ofs<<"  steering_wheel_cmd: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_cmd<<endl;
    ofs<<"  fl_steering_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_angle_actual<<endl;
    ofs<<"  fr_steering_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_angle_actual<<endl;
    ofs<<"  single_track_steering_angle: "<<vehicle_info.vehicle_info.lateral_data.single_track_steering_angle<<endl;
    ofs<<"  fl_steering_angle_expected: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_angle_expected<<endl;
    ofs<<"  fr_steering_angle_expected: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_angle_expected<<endl;
    ofs<<"  fl_steering_error: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_error<<endl;
    ofs<<"  fl_steering_cmd: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_cmd<<endl;
    ofs<<"  fr_steering_error: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_error<<endl;
    ofs<<"  fr_steering_cmd: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_cmd<<endl;
    ofs<<"  fl_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.fl_wheel_angular_velocity<<endl;
    ofs<<"  fr_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.fr_wheel_angular_velocity<<endl;
    ofs<<"  bl_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.bl_wheel_angular_velocity<<endl;
    ofs<<"  br_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.br_wheel_angular_velocity<<endl;
    ofs<<"  gas_percent: "<<vehicle_info.vehicle_info.longitudinal_data.gas_percent<<endl;
    ofs<<"  fl_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fl_gas_torque<<endl;
    ofs<<"  fr_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fr_gas_torque<<endl;
    ofs<<"  bl_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.bl_gas_torque<<endl;
    ofs<<"  br_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.br_gas_torque<<endl;
    ofs<<"  fl_brake_Torque: "<<vehicle_info.vehicle_info.longitudinal_data.fl_brake_Torque<<endl;
    ofs<<"  fr_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fr_brake_torque<<endl;
    ofs<<"  bl_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.bl_brake_torque<<endl;
    ofs<<"  br_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.br_brake_torque<<endl;
    ofs<<"}"<<endl;    

}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"ControlModule");
    Control Control;
    Control.Spin();
    return 0;
}
