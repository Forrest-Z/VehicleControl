#include <iostream>
using namespace std;
#include <car_demo/Control.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ControlModule");
    Control Control;
    Control.Spin();
    return 0;
}