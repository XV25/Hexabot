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
# if__name__ == "__main__":
    
#     pass





if __name__ == "__main__":
    rospy.init_node("crackMapping", anonymous=False, log_level=rospy.DEBUG)
    rospy.sleep(1)
    # scan = rospy.Subscriber("/phantomx/", LaserScan, lidarRead)
    # cmd = rospy.Publisher("/phantomx/cmd_vel",Twist, queue_size=5)
    while not rospy.is_shutdown():
        rospy.sleep(1)
    cameraModel = image_geometry.PinholeCameraModel()
    p_world = PointStamped()
    # p_world.header.seq = self.camera_image.header.seq
    p_world.header.stamp = stamp
    p_world.header.frame_id = '/world'
    p_world.point.x = p_3d[0]
    p_world.point.y = p_3d[1]
    p_world.point.z = p_3d[2]
    
    listener = tf.TransformListener()
    listener.waitForTransform('/base_link', '/world', stamp, rospy.Duration(1.0))
    p_camera = listener.transformPoint('/base_link', p_world)

#     # The navigation frame has X pointing forward, Y left and Z up, whereas the
# # vision frame has X pointing right, Y down and Z forward; hence the need to
# # reassign axes here.
#     x = -p_camera.point.y
#     y = -p_camera.point.z
#     z = p_camera.point.x

    # camera = image_geometry.PinholeCameraModel()
    # camera.fromCameraInfo(camera_info)
#     p_image = camera.project3dToPixel((x, y, z))

