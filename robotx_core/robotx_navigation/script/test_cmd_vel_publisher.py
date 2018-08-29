#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('test_cmd_vel_publisher', anonymous=False)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 1
        pub.publish(twist_msg)
        r.sleep()
