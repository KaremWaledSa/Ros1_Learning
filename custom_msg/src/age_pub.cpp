#include "ros/ros.h"
#include "custom_msg/age.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "age_pub");

    ros::NodeHandle n;

    ros::Publisher string_pub;

    string_pub = n.advertise<custom_msg::age>("custom_msg", 50);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {

        custom_msg::age msg;

        msg.Year = 1999;
        msg.Month = 5;
        msg.Day = 1;

        ROS_INFO("%f %f %f",msg.Year, msg.Month, msg.Day);

        string_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}