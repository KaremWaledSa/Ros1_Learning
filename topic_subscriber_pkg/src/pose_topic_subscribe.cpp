#include "ros/ros.h"
#include "turtlesim/Pose.h"


void TurtleCallBack(const turtlesim::Pose::ConstPtr& msg)
{
  ROS_INFO("Turtle X is: [%f] , Y is : [%f] , Theta is :[%f] ", msg->x ,msg->y ,msg->theta);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Sub_Turtle");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("turtle1/pose", 1000, TurtleCallBack);

  ros::spin();

  return 0;
}