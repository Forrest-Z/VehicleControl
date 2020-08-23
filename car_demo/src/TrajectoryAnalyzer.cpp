#include <car_demo/TrajectoryAnalyzer.h>

Status TrajectoryAnalyzer::Init(const ControlConf &control_conf)
{
    Ts=control_conf.conf_param.Ts;
    search_length=control_conf.conf_param.search_length;
    
    status.status = "OK";
    return status;   
}

Status TrajectoryAnalyzer::ReadTrajectory(const prius_msgs::My_Trajectory &trajectory)
{
    for(prius_msgs::My_Trajectory_Point point : trajectory.trajectory_points)
    {
        trajectory_info.push_back(point);
    }
    destination = trajectory_info.back();
    status.status = "OK";
    return status;
}
double TrajectoryAnalyzer::ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point, double x, double y)
{
    float distance;
    return distance = sqrt((trajectory_point.x-x)*(trajectory_point.x-x)+(trajectory_point.y-y)*(trajectory_point.y-y));
}

Status TrajectoryAnalyzer::MatchPointByPosition(VehicleState &vehicle_state)
{
    //ROS_INFO("start to find nearest point");
    float x=vehicle_state.movement_state.pose.position.x;
    float y=vehicle_state.movement_state.pose.position.y;
    
    vector<float> dist;
    int i =goal_id;
    while(i<goal_id+search_length&&i<trajectory_info.size())
    {
        float point_distance;
        point_distance = ComputeDist(trajectory_info[i],x,y);
        dist.push_back(point_distance);
        i++;
    }
    vector<float>::iterator goal=min_element(dist.begin(), dist.end());
    vehicle_state.distance_error = *goal;
    goal_id=distance(dist.begin(), goal)+goal_id;
    goal_state = trajectory_info[goal_id];

    status.status = "OK";
    return status;   
}
double TrajectoryAnalyzer::MatchPointByPositionForStanley(double x,double y)
{
    //ROS_INFO("start to find nearest point");
    vector<float> dist;
    int i =stanley_goal_id;
    while(i<stanley_goal_id+search_length&&i<trajectory_info.size())
    {
        float point_distance;
        point_distance = ComputeDist(trajectory_info[i],x,y);
        dist.push_back(point_distance);
        i++;
    }
    vector<float>::iterator goal=min_element(dist.begin(), dist.end());
    stanley_goal_id=distance(dist.begin(), goal)+stanley_goal_id ;
    return stanley_goal_id;
}
void TrajectoryAnalyzer::GetPreviewPoint(double preview_length)
{
    if(goal_id + preview_length<trajectory_info.size())
    {
        preview_id = goal_id + preview_length;
    }
    else
    {
        preview_id = trajectory_info.size();
    }
    
    preview_state = trajectory_info[preview_id];
}
vector<prius_msgs::My_Trajectory_Point> TrajectoryAnalyzer::GetPreviewTrajectory(double preview_length)
{
    GetPreviewPoint(preview_length);
    vector<prius_msgs::My_Trajectory_Point>  preview_trajectory;
    int range;
    if(goal_id + preview_length<trajectory_info.size())
    {
        range = preview_length;
    }
    else
    {
        range = trajectory_info.size()- goal_id;
    }
    for(int i=1;i<=range;i++)
    {
        preview_trajectory.push_back(trajectory_info[goal_id+i]);
        

    }
    //ROS_INFO_STREAM("The size of LQR preview trajectory is "<<preview_trajectory.size());
    return preview_trajectory;
}
void TrajectoryAnalyzer::PrintTrajectory()
{
    //ROS_INFO("Publish the trajectory!");
    for(int i =trajectory_info.size()-1 ;i>100;i=i-1000)
    {
        prius_msgs::My_Trajectory_Point point = trajectory_info[i];
        ROS_INFO_STREAM("x="<< point.x);
        ROS_INFO_STREAM("y=" << point.y);
        ROS_INFO_STREAM("z=" << point.z);
    }
}