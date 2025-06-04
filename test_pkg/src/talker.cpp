#include "ros/ros.h" 
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker"); // node name 
 

  ros::NodeHandle n; // access point to communicate with ros system

  
  ros::Publisher string_pub; // create object from ros class from Publisher type named string_pub

    string_pub= n.advertise<std_msgs::String>("chatter", 50); // publish data from type string of std_msgs , chatter is the topic name , and queue size 

  ros::Rate loop_rate(100); // publishing rate to the topic   

    
  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
 
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str()); // to print on terminal while node is working 


  string_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}