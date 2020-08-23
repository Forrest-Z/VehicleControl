#include<iostream>
using namespace std;
#include<ros/ros.h>
#include<fstream>
#include<vector>
#include "nox_msgs.h"
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>


const int LOCALIZATION_FREQUENCY = 100;
const double STOP_THRESHOLD =0.05;
const int UNIT_TIME_LENGTH=1;

class LocalPlanner
{
    public:
    LocalPlanner()
    {
        ontime_trajectory_pub=node.advertise<prius_msgs::My_Trajectory>("ontime_trajectory",10);
        ROS_INFO("ontime_trajectory_pub");
        global_trajectory_sub=node.subscribe("/global_trajectory",10,&LocalPlanner::TrajectoryCallback,this);
        ROS_INFO("global_trajectory_sub");
        localization_sub=node.subscribe("/base_pose_ground_truth",10,&LocalPlanner::LocalPlanning,this);
        ROS_INFO("localization_sub");
    }

    void TrajectoryCallback(const prius_msgs::My_Trajectory &routing);
    void LocalPlanning(const nav_msgs::Odometry &current_localization);
    void MatchPointByPosition(double x,double y);
    double ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point, double x, double y);
    nox_msgs::TrajectoryPoint TransformToVehicleFrame(nox_msgs::TrajectoryPoint point);
    nox_msgs::TrajectoryPoint Adapter(const prius_msgs::My_Trajectory_Point point);
    void ComputeOrientation();
    int CheckStopTime();

    private:
    int frequency = 10;
    int count=0;
    int stop_count=0;
    int stop_time=0;
    ros::NodeHandle node;
    ros::Publisher ontime_trajectory_pub;
    ros::Subscriber global_trajectory_sub;
    ros::Subscriber localization_sub;
    prius_msgs::My_Trajectory_Point destination;
    nav_msgs::Odometry localization;
    int goal_id=0;
    int search_length=500;
    vector<prius_msgs::My_Trajectory_Point> global_trajectory;
    int local_trajectory_length = 200;
    
    double pose_angle[3]={0};
    double heading_angle=0.0;
    double localization_x;
    double localization_y;
    double localization_z;

};
void LocalPlanner::TrajectoryCallback(const prius_msgs::My_Trajectory &routing)
{
    ROS_INFO("read path");
    for(prius_msgs::My_Trajectory_Point point : routing.trajectory_points)
    {
        global_trajectory.push_back(point);
    }
    
    destination = global_trajectory.back();
    ROS_INFO("read path successfully");
    for(int i =global_trajectory.size()-1 ;i>0;i=i-1000)
    {
        prius_msgs::My_Trajectory_Point point = global_trajectory[i];
        ROS_INFO_STREAM("x="<< point.x);
        ROS_INFO_STREAM("y="<< point.y);
        ROS_INFO_STREAM("z="<< point.z);
    }
    ROS_INFO("read path end");
    
}

void LocalPlanner::LocalPlanning(const nav_msgs::Odometry &current_localization)
{
    ROS_INFO_STREAM("stop_count is "<< stop_count);
    ROS_INFO_STREAM("stop_time is "<< stop_time);
    if(stop_count<stop_time*UNIT_TIME_LENGTH)
    {
        ++stop_count;
        
        return;
    }
    if(count<LOCALIZATION_FREQUENCY/frequency)
    {
        ++count;
        
        return;
    }
    ROS_INFO_STREAM("count is "<< count);
    if(global_trajectory.empty())
    {
        return;
    }
    count = 0;
    ROS_INFO("get localization");
    localization = current_localization;
    localization_x = localization.pose.pose.position.x;
    localization_y = localization.pose.pose.position.y;
    localization_z = localization.pose.pose.position.z;

    ComputeOrientation();
    ROS_INFO("match point");
    MatchPointByPosition(localization_x,localization_y);
    //ROS_INFO("read localization");


    
    if(global_trajectory[goal_id].v<STOP_THRESHOLD)
    {
        stop_time = CheckStopTime();
    }
    if(stop_count == stop_time*UNIT_TIME_LENGTH && stop_time>0)
    {
        stop_count = 0;
        stop_time = 0;
    }
    

    prius_msgs::My_Trajectory local_trajectory;
    vector<prius_msgs::My_Trajectory_Point>::iterator start =global_trajectory.begin()+goal_id;
    for(vector<prius_msgs::My_Trajectory_Point>::iterator it=start;it!=start+local_trajectory_length&&it!=global_trajectory.end();it++)
    {
        local_trajectory.trajectory_points.push_back(*it);
    }
    ontime_trajectory_pub.publish(local_trajectory);
    ROS_INFO("publish ontime trajectory");

    ROS_INFO_STREAM("current x is "<<localization_x);
    ROS_INFO_STREAM("current y is "<<localization_y);
    ROS_INFO_STREAM("the size of ontime trajectory is "<<local_trajectory.trajectory_points.size());
    ROS_INFO_STREAM("local trajectory first x is "<<local_trajectory.trajectory_points[0].x);
    ROS_INFO_STREAM("local trajectory first y is "<<local_trajectory.trajectory_points[0].y);
    ROS_INFO_STREAM("local trajectory last x is "<<local_trajectory.trajectory_points.back().x);
    ROS_INFO_STREAM("local trajectory last y is "<<local_trajectory.trajectory_points.back().y);
}

int LocalPlanner::CheckStopTime()
{
    int i = goal_id;
    while(true)
    {
        if(global_trajectory[i].v<STOP_THRESHOLD)
        {
            i++;
        }
        else
        {
            break;
        }
    }
    int stop_time = i-goal_id;
    goal_id = i+30;
    return stop_time;

}
void LocalPlanner::ComputeOrientation()
{
    float x = localization.pose.pose.orientation.x;
    float y = localization.pose.pose.orientation.y;
    float z = localization.pose.pose.orientation.z;
    float w = localization.pose.pose.orientation.w;
    double roll = atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z);
    double pitch = asin(-2 * (x*z - w*y));
    double yaw = atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z);
    pose_angle[0]=roll;
    pose_angle[1]=yaw;
    pose_angle[2]=pitch;
    heading_angle = pose_angle[1];
}
void LocalPlanner::MatchPointByPosition(double x,double y)
{
    //ROS_INFO("start to find nearest point");    
    vector<double> dist;
    int i =goal_id;
    //ROS_INFO("start read");
    while(i<goal_id+search_length&&i<global_trajectory.size())
    {
        double point_distance;
        point_distance = ComputeDist(global_trajectory[i],x,y);
        dist.push_back(point_distance);
        i++;
    }
    //ROS_INFO("read finished");
    vector<double>::iterator goal=min_element(dist.begin(), dist.end());
    //ROS_INFO("find min");
    goal_id=distance(dist.begin(), goal)+goal_id;
    //ROS_INFO("get id");
}
double LocalPlanner::ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point, double x, double y)
{
    float distance;
    double point_x = trajectory_point.x;
    double point_y = trajectory_point.y;

    return distance = sqrt((point_x-x)*(point_x-x)+(point_y-y)*(point_y-y));
}





nox_msgs::TrajectoryPoint LocalPlanner::Adapter(const prius_msgs::My_Trajectory_Point point)
{
    nox_msgs::TrajectoryPoint nox_point;
    nox_point.a=point.a;
    nox_point.t=point.relative_time;
    nox_point.v=point.v;
    nox_point.info.kappa=point.kappa;
    nox_point.info.s=point.s;
    nox_point.info.pose.position.x=point.x;
    nox_point.info.pose.position.y=point.y;
    nox_point.info.pose.position.z=point.z;
    return nox_point;
}

nox_msgs::TrajectoryPoint LocalPlanner::TransformToVehicleFrame(nox_msgs::TrajectoryPoint point)
{
    nox_msgs::TrajectoryPoint point_in_vehicle_frame=point;
    double diff_x=point.info.pose.position.x-localization_x;
    double diff_y=point.info.pose.position.y-localization_y;
    point_in_vehicle_frame.info.pose.position.x=diff_y*sin(heading_angle)+diff_x*cos(heading_angle);
    point_in_vehicle_frame.info.pose.position.y=diff_y*cos(heading_angle)-diff_x*sin(heading_angle);
    point_in_vehicle_frame.info.pose.position.z=point.info.pose.position.x-localization_z;
    return point_in_vehicle_frame;
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planning");
  LocalPlanner LocalPlanner;
  ros::spin();
  return 0;
}