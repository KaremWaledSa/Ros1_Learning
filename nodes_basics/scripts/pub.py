#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter_in_Py', String, queue_size=10)
    rospy.init_node('Pub_Py', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world " + "&karem" 
        rospy.loginfo(hello_str)
        pub.publish(hello_str )
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass