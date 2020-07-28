#include <car_demo/VehicleState.h>

Status VehicleState::Init(ControlConf &control_conf)
{
    Ts=control_conf.conf_param.Ts;
    slope_threshold=control_conf.conf_param.slope_threshold;
    G=control_conf.conf_param.G;
    mass=control_conf.conf_param.mass;
    friction_coefficient=control_conf.conf_param.friction_coefficient;
    wheel_base=control_conf.conf_param.wheel_base;
    front_wheel_base=control_conf.conf_param.front_wheel_base;
    rear_wheel_base=control_conf.conf_param.rear_wheel_base;
    front_track=control_conf.conf_param.front_track;
    rear_track=control_conf.conf_param.rear_track;
    wheel_radius=control_conf.conf_param.wheel_radius;
    Ix=control_conf.conf_param.Ix;
    Iy=control_conf.conf_param.Iy;
    Iz=control_conf.conf_param.Iz;
    brake_distance_coefficient=control_conf.conf_param.brake_distance_coefficient;
    chassis_aero_force_gain=control_conf.conf_param.chassis_aero_force_gain;
    max_front_torque=control_conf.conf_param.max_front_torque;
    max_back_torque=control_conf.conf_param.max_back_torque;
    max_front_brake_torque=control_conf.conf_param.max_front_brake_torque;
    max_back_brake_torque=control_conf.conf_param.max_back_brake_torque;
    max_speed=control_conf.conf_param.max_speed;
    max_steer=control_conf.conf_param.max_steer;
    gas_efficiency=control_conf.conf_param.gas_efficiency;
    battery_charge_watt_hours=control_conf.conf_param.battery_charge_watt_hours;
    battery_discharge_watt_hours=control_conf.conf_param.battery_discharge_watt_hours;
    flwheel_steering_p_gain=control_conf.conf_param.flwheel_steering_p_gain;
    frwheel_steering_p_gain=control_conf.conf_param.frwheel_steering_p_gain;
    flwheel_steering_i_gain=control_conf.conf_param.flwheel_steering_i_gain;
    frwheel_steering_i_gain=control_conf.conf_param.frwheel_steering_i_gain;
    flwheel_steering_d_gain=control_conf.conf_param.flwheel_steering_d_gain;
    frwheel_steering_d_gain=control_conf.conf_param.frwheel_steering_d_gain;
    status.status = "OK";
    return status;
}

Status VehicleState::GetVehicleStateFromLocalization(const nav_msgs::Odometry &localization)
{
    movement_state.pose =  localization.pose.pose;
    ros::param::set("vehicle_x", movement_state.pose.position.x);
    ros::param::set("vehicle_y", movement_state.pose.position.y);
    movement_state.twist =  localization.twist.twist;
    ComputeVelocity();
    //ROS_INFO_STREAM("Current velocity(from chassis) is "<<info_from_chassis.longitudinal_data.vel_from_localization);
    ComputeAcc();
    //ROS_INFO_STREAM("Current acceleration(from chassis) is "<<info_from_chassis.longitudinal_data.acceleration);
    ComputeHeadingAngle();
    //ROS_INFO_STREAM("Current heading(from chassis) is "<<info_from_chassis.lateral_data.heading_angle);
    status.status = "OK";
    return status;
}

Status VehicleState::GetVehicleStateFromChassis(const prius_msgs::VehicleInfo &current_state)
{
    info_from_chassis = current_state;
    status.status = "OK";
    return status;
}
void VehicleState::GetAllData(const prius_msgs::My_Trajectory_Point &goal_state, const prius_msgs::My_Trajectory_Point &preview_state,const ControlConf &control_conf)
{
/*     vehicle_info.controllers_num=control_conf.conf_param.controller_num;
    vehicle_info.longitudinal_controller=control_conf.LONGITUDINAL_CONTROLLER;
    vehicle_info.lateral_controller=control_conf.LATERAL_CONTROLLER;
    vehicle_info.centralized_controller=control_conf.CENTRALIZED_CONTROLLER; */
    vehicle_info.distance_error = distance_error;
    vehicle_info.velocity_error = velocity_error;
    vehicle_info.heading_error = heading_error;
    vehicle_info.goal_x=goal_state.x;
    vehicle_info.goal_y=goal_state.y;
    vehicle_info.preview_x= preview_state.x;
    vehicle_info.preview_y=preview_state.y;
    vehicle_info.trajectory_point.x=movement_state.pose.position.x;
    vehicle_info.trajectory_point.y=movement_state.pose.position.y;
    vehicle_info.trajectory_point.v=current_velocity;
    vehicle_info.trajectory_point.kappa=goal_state.kappa;
    vehicle_info.trajectory_point.theta = heading_angle;
    vehicle_info.trajectory_point.relative_time=ros::Time::now().toSec();
    vehicle_info.trajectory_point.a=acceleration;
    vehicle_info.vehicle_info = info_from_chassis;
}

void VehicleState::ComputeDistanceFromDestination(prius_msgs::My_Trajectory_Point destination)
{
    double x = movement_state.pose.position.x;
    double y = movement_state.pose.position.y;
    this->distance_from_destination = sqrt((destination.x-x)*(destination.x-x)+(destination.y-y)*(destination.y-y));
    //ROS_INFO_STREAM("Current distance from destination(from GPS) is "<<distance_from_destination);
}
void VehicleState::ComputeVelocity()
{
    current_velocity= sqrt(movement_state.twist.linear.x*movement_state.twist.linear.x+movement_state.twist.linear.y*movement_state.twist.linear.y);
    ros::param::set("vehicle_v", current_velocity);
    //ROS_INFO_STREAM("Current velocity(from GPS) is "<<current_velocity);
}


void VehicleState::ComputeAcc()
{
    acceleration= (current_velocity-previous_velocity)/Ts;
    previous_velocity = current_velocity;
    ros::param::set("current_a", current_velocity);
    //ROS_INFO_STREAM("Current acceleration(from GPS) is "<<acceleration);
}
void VehicleState::ComputeEular(double x,double y,double z,double w)
{
    double roll = atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z);
    double pitch = asin(-2 * (x*z - w*y));
    double yaw = atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z);
    pose_angle[0]=roll;
    pose_angle[1]=yaw;
    pose_angle[2]=pitch;

}
void VehicleState::ComputeHeadingAngle()
{
    float x = movement_state.pose.orientation.x;
    float y = movement_state.pose.orientation.y;
    float z = movement_state.pose.orientation.z;
    float w = movement_state.pose.orientation.w;
    ComputeEular(x,y,z,w);
    heading_angle = pose_angle[1];
    //ROS_INFO_STREAM("Current heading angle(from GPS) is "<<heading_angle);
    ros::param::set("Lattice_Planner", heading_angle);
}

void VehicleState::ComputeBrakeDistanceAhead()
{
    brake_distance_ahead = current_velocity*brake_distance_coefficient;
/*     ROS_INFO_STREAM("current velocity is "<<current_velocity);
    ROS_INFO_STREAM("mass is "<<mass);
    ROS_INFO_STREAM("wheel_radius is "<<wheel_radius);
    ROS_INFO_STREAM("max_front_brake_torque is "<<max_front_brake_torque);
    ROS_INFO_STREAM("max_back_brake_torque is "<<max_back_brake_torque); */
}

void VehicleState::ComputeSlope()
{
/*     float qx = current_state.pose.orientation.x;
    float qy = current_state.pose.orientation.y;
    float qz = current_state.pose.orientation.z;
    float qw = current_state.pose.orientation.w;

    // euler = euler_from_quaternion(qx,qy,qz,qw);
    // float pitch= euler[2];
    // 这里随便定一个一个pitch，实际用欧拉角公式求，还没写
    float pitch = 0.0; 
    pitch_angle.pop_back();
    pitch_angle.push_front(pitch);
    if (pitch>=SLOPE_THRESHOLD)
    {
        count_slope +=1;
    }
    else
    {
        count_flat += 1;
    }

    if(count_slope+count_flat==N)
    {
        if(count_slope>N/2)
        {
            slope = accumulate(begin(pitch_angle),end(pitch_angle),0.0)/N;
            previous_slope = slope;
            count_slope=0;
            count_flat=0;
            return slope;
        }
        else
        {
            count_slope=0;
            count_flat=0;
            return 0.0;
        }
    }
    else
    {
        return previous_slope;
    } */
}