#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import copy

from robotx_msgs.msg import ObjectRegionOfInterestArray
from prediction_module import Predicdion

class PredicdionNode:

    def __init__(self):
        self.predicdion = Predicdion()
        self.bridge = CvBridge()
        image_sub = rospy.Subscriber("wam_v/front_camera/front_image_raw", Image,self.image_callback,queue_size=1)
        roi_sub = rospy.Subscriber("object_bbox_extractor_node/object_roi", ObjectRegionOfInterestArray,self.roi_callback,queue_size=1)
        self.recognition_pub = rospy.Publisher('recognition_result', Float32MultiArray, queue_size=1)
        self.prediction_result = Float32MultiArray()

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

        classes=['green', 'other', 'red', 'white']
        colors=[(152,251,152), (0,255,255), (203,192,255), (80,80,80)]
        font = cv2.FONT_HERSHEY_PLAIN
        cv_image_ = copy.deepcopy(cv_image)

        for roi in rois:

            left = roi.roi_2d.x_offset
            top = roi.roi_2d.y_offset
            right = left + roi.roi_2d.width
            bottom = top + roi.roi_2d.height            
            roi_image = cv_image[top:bottom, left:right]

            cnn_output = self.predicdion(roi_image)

            index = np.argmax(cnn_output)
            # str_label = classes[index]
            # print(str_label)
            # print(cnn_output)

            cv2.rectangle(cv_image_, (left, top), (right, bottom), colors[index], thickness=4, lineType=cv2.LINE_4)

            # self.prediction_result.data = [0.0,0.0,0.0,1.0]
            # self.prediction_result.data = list(cnn_output)

        cv2.imshow('subscribed_image',cv_image_)
        cv2.waitKey(3)

        # self.recognition_pub.publish(self.prediction_result)
        # rospy.loginfo("prediction result is published.")


if __name__ == '__main__':
    cnn_prediction_node = PredicdionNode()
    rospy.init_node('cnn_prediction', anonymous=True)
    rospy.spin()
