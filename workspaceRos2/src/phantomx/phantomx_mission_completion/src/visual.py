#!/usr/bin/env python
# -*- coding: utf-8

from crackDetection import *


from sensor_msgs.msg import Image as ImageMSG
from sensor_msgs.msg import CameraInfo as CamInfoMSG
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import image_geometry
import tf
from tf2_geometry_msgs import PointStamped
import time


from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# if__name__ == "__main__":

#     pass
class test_mark():
    def __init__(self):
        rospy.init_node("test", anonymous=True)
        self.pub = rospy.Publisher('state', PointStamped, queue_size=10)
        self.markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)

        def pointToMarker(self,fissures):
            self.liste_marker = MarkerArray()
            for fissure in  fissures:
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.get_rostime()
                marker.type = 2
                marker.pose.position.x = sefissure.point.x
                marker.pose.position.y = self.fissure.point.y
                marker.pose.position.z = self.fissure.point.z
                marker.pose.position.x = self.fissure.point.x
                marker.pose.position.y = self.fissure.point.y
                marker.pose.position.z = self.fissure.point.z
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.marker.lifetime = rospy.Duration(0)
                liste_marker.append(marker)
            return liste_marker




        self.fissure = PointStamped()
        self.fissure.point.x = 1
        self.fissure.point.y = 2
        self.fissure.point.z = 3

        self.fissure_2 = PointStamped()
        self.fissure_2.point.x = 3
        self.fissure_2.point.y = 3
        self.fissure_2.point.z = 3



        self.marker = Marker()
        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.type = 6
        self.marker.pose.position.x = self.fissure_2.point.x
        self.marker.pose.position.y = self.fissure_2.point.y
        self.marker.pose.position.z = self.fissure_2.point.z

        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1

        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration(0)

        # On definit les fonctiond de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes




if __name__ == "__main__":
    #while not rospy.is_shutdown():
    mark = test_mark()
    mark.markerPub.publish(mark.marker)
    print ("sending marker", mark.marker)
    rospy.sleep(0.1)


    # scan = rospy.Subscriber("/phantomx/", LaserScan, lidarRead)
