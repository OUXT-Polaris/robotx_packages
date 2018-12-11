#!/usr/bin/env python
import rospy
from robotx_msgs.msg import ObjectRegionOfInterestArray
from geometry_msgs.msg import PoseStamped

class field_map_builder:
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/cnn_prediction_node/object_roi", ObjectRegionOfInterestArray, self.bbox_callback)
        self.robot_pose_callback = rospy.Subscriber("/robot_pose", PoseStamped, self.robot_pose_callback)
    def bbox_callback(self,msg):
        print(msg)
    def robot_pose_callback(self,msg):
        print(msg)

if __name__ == "__main__":
    rospy.init_node('gps_plotter_node')
    builder =  field_map_builder()
    rospy.spin()