#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from robotx_msgs.msg import Objects2D
from cv_bridge import CvBridge, CvBridgeError
import cv2

class objects2d_viewer:
    def __init__(self):
        self.image_pub = rospy.Publisher(rospy.get_name()+"/detected_image",Image)
        self.bridge = CvBridge()
        self.image_recieved = False
        self.image_sub = rospy.Subscriber(rospy.get_name()+"/image_raw",Image,self.image_callback)
        self.detection_result_sub = rospy.Subscriber(rospy.get_name()+"/objects_2d",Objects2D,self.detection_result_callback)

    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_recieved = True
        except CvBridgeError as e:
            print(e)
    def detection_result_callback(self,data):
        if self.image_recieved == True:
            for detected_object in data.objects:
                x0 = int(detected_object.boundingbox.corner_point_0[0])
                y0 = int(detected_object.boundingbox.corner_point_0[1])
                x1 = int(detected_object.boundingbox.corner_point_1[0])
                y1 = int(detected_object.boundingbox.corner_point_1[1])
                self.cv_image = cv2.rectangle(self.cv_image,(x0,y0),(x1,y1),(0,0,255),1)
                cv2.putText(self.cv_image, detected_object.type, (int((x0+x1)/2), y0-10),
                    cv2.FONT_HERSHEY_PLAIN, 1.,(0,0,255), 1, cv2.LINE_AA)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

if __name__ == '__main__':
    rospy.init_node('objects2d_viewer', anonymous=True)
    viewer = objects2d_viewer()
    rospy.spin()
