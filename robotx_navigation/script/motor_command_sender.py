#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from robotx_msgs.msg import UsvDrive

class motor_command_sender:
    def __init__(self):
        self.left_motor_cmd = 0
        self.right_motor_cmd = 0
        self.left_motor_cmd_recieved = False
        self.right_motor_cmd_recieved = False
        self.usv_drive_pub = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=1)
        self.left_motor_cmd_sub = rospy.Subscriber('/left_motor/cmd',Float64,self.left_motor_callback)
        self.right_motor_cmd_sub = rospy.Subscriber('/right_motor/cmd',Float64,self.right_motor_callback)
    def left_motor_callback(self,msg):
        self.left_motor_cmd = msg.data
        self.left_motor_cmd_recieved = True
        if self.left_motor_cmd_recieved == True and self.right_motor_cmd_recieved == True:
            usv_drive_msg = UsvDrive()
            usv_drive_msg.left = self.left_motor_cmd
            usv_drive_msg.right = self.right_motor_cmd
            self.usv_drive_pub.publish(usv_drive_msg)
    def right_motor_callback(self,msg):
        self.right_motor_cmd = msg.data
        self.right_motor_cmd_recieved = True
        if self.left_motor_cmd_recieved == True and self.right_motor_cmd_recieved == True:
            usv_drive_msg = UsvDrive()
            usv_drive_msg.left = self.left_motor_cmd
            usv_drive_msg.right = self.right_motor_cmd
            self.usv_drive_pub.publish(usv_drive_msg)

if __name__ == '__main__':
    rospy.init_node('motor_command_sender', anonymous=True)
    sender = motor_command_sender()
    rospy.spin()
