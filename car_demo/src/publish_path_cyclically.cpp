#include<iostream>
using namespace std;
#include<ros/ros.h>
#include<fstream>
#include<vector>
#include<prius_msgs/Control.h>
#include<prius_msgs/My_Trajectory.h>

#define LONGITUDINAL_TEST_DATA "/home/ht/pruis/src/car_demo/src/datatest/longitudinal_controller_test/longitudinal_trajectory_cyclically.txt"
int PATH_LENGTH=20;

float get_num(string s)
{
    int index = s.find(":");
    int size;
    size = s.size();
    string ss;
    ss = s.substr(index + 2, size - 1);
    istringstream iss(ss);
    float num;
    iss >> num;
    return num;
}

int read_from_file(prius_msgs::My_Trajectory &my_trajectory, ifstream& ifs)
{

    string s;

    for(int i = 0;i<PATH_LENGTH;i++)
    {
        prius_msgs::My_Trajectory_Point point;
        double* p = &(point.x);
        while (true)
        {
            getline(ifs,s);
            int index = s.find(":");
            int size;
            double num;
            string data;
            if ( index != string::npos)
            {
                size = s.size();
                data = s.substr(index + 2, size-index-2);
                istringstream iss(data);
                iss >> num;
                (*p++) = num;
            }
            else if(s[0]=='}')
            {
                break;
            }
        }
        my_trajectory.trajectory_points.push_back(point);

    }
    return 0;
}

void print_data(prius_msgs::My_Trajectory &my_trajectory)
{
    cout<<my_trajectory.header.stamp<<endl;
    cout << my_trajectory.header.seq << endl;
    prius_msgs::My_Trajectory_Point point = my_trajectory.trajectory_points[0];
    cout << "x="<< point.x << endl;
    cout << "y=" << point.y << endl;
    cout << "z=" << point.z << endl;
    cout << "theta=" << point.theta << endl;
    cout << "kappa=" << point.kappa << endl;
    cout << "s=" << point.s << endl;
    cout << "dkappa=" << point.dkappa << endl;
    cout << "v=" << point.v << endl;
    cout << "a=" << point.a << endl;
    cout << "relative_time=" << point.relative_time << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_publisher");
    ros::NodeHandle node;
    ros::Publisher trajectory_pub=node.advertise<prius_msgs::My_Trajectory>("/prius/planning_output",10);
    ros::Rate loop_rate(1);

    ifstream ifs;
    ifs.open(LONGITUDINAL_TEST_DATA, ios::in);
    if (!ifs.is_open())
    {
        cout << "读取失败" << endl;
        return 0;
    }

    int count = 0;
    while(ros::ok())
    {
        prius_msgs::My_Trajectory my_trajectory;
        read_from_file(my_trajectory,ifs);
        trajectory_pub.publish(my_trajectory);
        ROS_INFO_STREAM("publish the "<<count+1<<" trajectory");
        print_data(my_trajectory);
        loop_rate.sleep();
        count++;
    }
    return 0;
}
