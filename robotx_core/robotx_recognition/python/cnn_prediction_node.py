#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import copy

from robotx_msgs.msg import ObjectRegionOfInterestArray
from robotx_msgs.msg import ObjectType

from prediction_module import Predicdion


class PredicdionNode:

    def __init__(self):
        self.predicdion = Predicdion()
        self.bridge = CvBridge()
        image_sub = rospy.Subscriber("wam_v/front_camera/front_image_raw", Image,self.image_callback,queue_size=1)
        roi_sub = rospy.Subscriber("object_bbox_extractor_node/object_roi", ObjectRegionOfInterestArray,self.roi_callback,queue_size=1)
        self.prediction_pub = rospy.Publisher('cnn_prediction_node/object_roi', ObjectRegionOfInterestArray, queue_size=1)
        self.object_dict = {}
        self.read_object_message()
        self.classes=['GREEN_BUOY', 'OTHER', 'RED_BUOY', 'WHITE_BUOY']


    def read_object_message(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("robotx_msgs")
        path = path + "/msg/ObjectType.msg"
        f = open(path)
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            line = line.split("uint8")[1]
            line = line.split("=")
            if len(line) == 2:
                line[0] = line[0].replace(" ","")
                self.object_dict.update({line[0]:int(line[1])})


    def image_callback(self,data):
        self.image_timestamp = data.header.stamp
        self.image_raw = data


    def roi_callback(self,data):

        rois = data.object_rois
        rois_timestamp = rois[0].header.stamp

        if not rois_timestamp == self.image_timestamp:
            # rospy.logwarn("timestamp is not synchronized.")
            return
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(self.image_raw, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.cv_image is None:
            rospy.logwarn("empty image is subscribed.")
            return

        self.buoy_recognition(self.cv_image, rois)


    def buoy_recognition(self, cv_image, rois):

        # colors=[(152,251,152), (0,255,255), (203,192,255), (80,80,80)]
        # font = cv2.FONT_HERSHEY_PLAIN
        # cv_image_ = copy.deepcopy(cv_image)

        prediction_result = copy.deepcopy(rois)

        for i in range(len(rois)):

            left = rois[i].roi_2d.x_offset
            top = rois[i].roi_2d.y_offset
            right = left + rois[i].roi_2d.width
            bottom = top + rois[i].roi_2d.height            
            roi_image = cv_image[top:bottom, left:right]

            cnn_output = self.predicdion(roi_image)

            index = np.argmax(cnn_output)
            prob = max(cnn_output)

            prediction_result[i].object_type.ID = self.object_dict[self.classes[index]]
            prediction_result[i].objectness = prob

        #     cv2.rectangle(cv_image_, (left, top), (right, bottom), colors[index], thickness=4, lineType=cv2.LINE_4)

        # cv2.imshow('subscribed_image',cv_image_)
        # cv2.waitKey(3)

        self.prediction_pub.publish(prediction_result)


if __name__ == '__main__':
    cnn_prediction_node = PredicdionNode()
    rospy.init_node('cnn_prediction', anonymous=True)
    rospy.spin()
