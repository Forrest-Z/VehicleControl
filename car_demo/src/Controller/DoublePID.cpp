#include <car_demo/Controller/DoublePID.h>

Status DoublePID::Init(ControlConf &control_conf)
{
    name = "Double PID controller";
    Ts=control_conf.conf_param.Ts;
    velocity_kp=control_conf.conf_param.velocity_kp;
    velocity_ki=control_conf.conf_param.velocity_ki;
    velocity_kd=control_conf.conf_param.velocity_kd;
    acc_kp=control_conf.conf_param.acc_kp;
    acc_ki=control_conf.conf_param.acc_ki;
    acc_kd=control_conf.conf_param.acc_kd;
    
    velocity_PID.Init(velocity_kp,velocity_ki,velocity_kd);
    acceleration_PID.Init(acc_kp,acc_ki,acc_kd);
    ROS_INFO_STREAM(name<<" is initialized successfully!");
    status.status = "OK";
    return status;

}
Status DoublePID::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Use DoublePID");

    vehicle_state.heading_error = trajectory_analyzer.goal_state.theta-vehicle_state.heading_angle;
    vehicle_state.velocity_error = trajectory_analyzer.goal_state.v-vehicle_state.current_velocity*cos(vehicle_state.heading_error);
/*     ROS_INFO_STREAM("velocity_error="<<vehicle_state.velocity_error);
    ROS_INFO_STREAM("distance_error="<<vehicle_state.distance_error);
    ROS_INFO_STREAM("heading_angle_error="<<vehicle_state.heading_error); */
    velocity_PID.control_increment(vehicle_state.velocity_error, acc_compensation,Ts);
    slope_compensation = vehicle_state.G*sin(vehicle_state.slope);
    // ROS_INFO_STREAM("acceleration compensation="<<acc_compensation);
    vehicle_state.acc_error=trajectory_analyzer.goal_state.a+acc_compensation+slope_compensation-vehicle_state.acceleration;
    //ROS_INFO_STREAM("acceleration error="<<vehicle_state.acc_error);
    acceleration_PID.control_increment(vehicle_state.acc_error,torque_cmd,Ts);
    //ROS_INFO_STREAM("torque_cmd="<<torque_cmd);
    torque_cmd /=vehicle_state.max_front_torque;
    //ROS_INFO_STREAM("torque_cmd="<<torque_cmd);
    if(torque_cmd>1)
    {
        torque_cmd=1;
    }
    else if(torque_cmd<-1)
    {
        torque_cmd = -1;
    }
    if(torque_cmd>0)
    {
        control_cmd.throttle = torque_cmd;
        control_cmd.brake = 0.0;
        ROS_INFO_STREAM("throttle="<<control_cmd.throttle);
    }
    else
    {
        control_cmd.throttle = 0.0;
        control_cmd.brake = -torque_cmd;
        ROS_INFO_STREAM("brake="<<control_cmd.brake);
    }

    control_cmd.shift_gears = control_cmd.FORWARD;
    ROS_INFO("Compute longitudinal command successfully!");

    status.status = "OK";
    return status;
}