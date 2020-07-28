#include<iostream>
using namespace std;
#include<ros/ros.h>
#include<fstream>
#include<vector>
#include<prius_msgs/Control.h>
#include<prius_msgs/My_Trajectory.h>

const double PI=3.14159236;
const int VELOCITY_FACTOR=2;


void FindAll(string s, char ch,vector<int> &pos)
{
    for(int i=0;i<s.length();i++)
    {
        if(s[i]==ch)
        {
            pos.push_back(i);
        }
    }
}
void GetData(int start,int end,string s,double &target)
{
    string data = s.substr(start,end-start);
    istringstream iss(data);
    iss >> target;   
}

int ReadTrueTrajectory(prius_msgs::My_Trajectory &my_trajectory, ifstream& ifs)
{
    string s;
    int i=0;
    while(getline(ifs,s))
    {
        
        prius_msgs::My_Trajectory_Point point;
        vector<int> pos;
        pos.push_back(-1);
        FindAll(s,'\t',pos);
        ROS_INFO("before");
        GetData(pos[0]+1,pos[1],s,point.x);
        GetData(pos[1]+1,pos[2],s,point.y);
        GetData(pos[3]+1,pos[4],s,point.theta);
        ROS_INFO("after");
        point.theta=point.theta*PI/180;
        GetData(pos[5]+1,pos[6],s,point.v);
        point.v=point.v*VELOCITY_FACTOR;
        my_trajectory.trajectory_points.push_back(point);
        ROS_INFO_STREAM("read "<<i++<<" point");
    }
    return 0;   
}

/* float GetNum(string s)
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
} */

int ReadFromFile(prius_msgs::My_Trajectory &my_trajectory, ifstream& ifs)
{

    string s;
    while(getline(ifs,s))
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

void PrintData(prius_msgs::My_Trajectory &my_trajectory, int index)
{
    cout<<my_trajectory.header.stamp<<endl;
    cout << my_trajectory.header.seq << endl;
    prius_msgs::My_Trajectory_Point point = my_trajectory.trajectory_points[index];
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
    cout<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_publisher");
    ros::NodeHandle node;
    ros::Publisher trajectory_pub=node.advertise<prius_msgs::My_Trajectory>("/prius/planning_output",10);

    string file_path;
    node.getParam("test_trajectory_path", file_path);

    //string file_path="/home/ht/ControlModule/src/car_demo/test_trajectories/recorded_trajectory_1_record.txt";

    int index=file_path.find(".");
    int index2=file_path.rfind("_",index);
    string trajectory_velocity;
    node.param<string>("controllers_num", trajectory_velocity, "");
    trajectory_velocity=file_path.substr(index2+1,index-index2-1);
    node.setParam("trajectory_velocity",trajectory_velocity);
    ROS_INFO_STREAM("velocity is "<<trajectory_velocity);


    int pos = file_path.rfind("/",index);
    int pos2= file_path.find("_",pos);
    string trajectory_type=file_path.substr(pos+1,pos2-pos-1);
    ROS_INFO_STREAM("trajectory_type is "<<trajectory_type);



    ifstream ifs;
    ifs.open(file_path, ios::in);
    if (!ifs.is_open())
    {
        cout << "读取失败" << endl;
        return 0;
    }

    prius_msgs::My_Trajectory my_trajectory;
    if(trajectory_type=="mat"||trajectory_type=="recorded")
    {
        ReadFromFile(my_trajectory,ifs);
    }
    else if(trajectory_type=="true")
    {
        ReadTrueTrajectory(my_trajectory,ifs);
    }
    
    


    sleep(3);
    trajectory_pub.publish(my_trajectory);


    int i = 0;
    while(i<1401)
    {
        cout<<"the "<<i<<" trajectory"<<endl;
        PrintData(my_trajectory,i);
        i=i+100;
    }

    return 0;
}
