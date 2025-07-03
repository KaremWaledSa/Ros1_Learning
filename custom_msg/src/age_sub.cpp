#include "ros/ros.h"
#include "custom_msg/age.h"

#include <sstream>

void chatterCallback(const custom_msg::age::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f %f %f]", msg->Year,msg->Month,msg->Day);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "age_sub");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("custom_msg", 1000, chatterCallback);

  ros::spin();

  return 0;
}