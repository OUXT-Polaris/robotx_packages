#!/usr/bin/env python
import utm
import rospy
import rospkg
import csv
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Vector3,Quaternion
import math
import tf

class gps_pose_plotter:
    def __init__(self):
        rospack = rospkg.RosPack()
        pose_csv_path = rospack.get_path('robotx_tools')+'/data/gps-pose.csv'
        markers = MarkerArray()
        pub = rospy.Publisher('gps_pose_marker', MarkerArray, queue_size=1, latch=True)
        with open(pose_csv_path, 'r') as f:
            reader = csv.reader(f)
            i = 0
            for row in reader:
                marker = Marker()
                label_marker = Marker()
                marker.header.frame_id = "world"
                label_marker.header.frame_id = "world"
                marker.ns = "pose"
                label_marker.ns = "description"
                label_marker.id = i
                marker.id = i
                label_marker.type = label_marker.TEXT_VIEW_FACING
                label_marker.scale.x = 10
                label_marker.scale.y = 10
                label_marker.scale.z = 10
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.scale.x = 10
                marker.scale.y = 5
                marker.scale.z = 10
                latlon = self.convert_to_latlon(row[1],row[2],row[3],row[4],row[5],row[6],row[7],row[8])
                utm_point = utm.from_latlon(latlon[0],latlon[1])
                label_marker.text = row[0]
                label_marker.pose.position.x = utm_point[0]
                label_marker.pose.position.y = utm_point[1]
                label_marker.pose.position.z = 10
                label_marker.color.r = 1
                label_marker.color.g = 1
                label_marker.color.b = 1
                label_marker.color.a = 1
                marker.pose.position.x = utm_point[0]
                marker.pose.position.y = utm_point[1]
                marker.color.b = 1
                marker.color.a = 1
                label_marker.frame_locked = True
                marker.frame_locked = True
                label_marker.pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, float(row[9])/180*math.pi))
                marker.pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, float(row[9])/180*math.pi))
                markers.markers.append(marker)
                markers.markers.append(label_marker)
                i = i + 1
        pub.publish(markers)
    def convert_to_latlon(self,lat_deg,lat_min,lat_sec,NorS,lon_deg,lon_min,lon_sec,EorW):
        lat = float(lat_deg) + float(lat_min)/60 + float(lat_sec)/3600
        lon = float(lon_deg) + float(lon_min)/60 + float(lon_sec)/3600
        if(NorS=='N'):
            lat = lat
        else:
            lat = -lat
        if(EorW=='E'):
            lon = lon
        else:
            lon = -lon
        return [lat,lon]
    def euler_to_quaternion(self,euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

if __name__ == '__main__':
    rospy.init_node('gps_pose_plotter_node')
    plotter = gps_pose_plotter()
    rospy.spin()