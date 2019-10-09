#include <ros/ros.h>
#include "cplus_xsens_driver/MtDevice.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cxsens_driver");
    xsens_device::MTDevice driver;
    ros::spin(); 
    return 0;
}
