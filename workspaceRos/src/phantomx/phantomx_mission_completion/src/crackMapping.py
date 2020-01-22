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
# if__name__ == "__main__":
    
#     pass


class CrackMap():
    
    # On definit les fonctiond de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes
       
    def callbackCIFrontDepth(self,img):
	    self.CIFrontDepth = img
	    
    def callbackCIFrontColor(self,img):
	    self.CIFrontColor = img
	     
        
    def fromPix2camRef(self,pixD):
        """
        pixD : (u,v,depth of pix)
        """
        
         # récup du topic
        self.cameraModel.fromCameraInfo(self.CIFrontColor)
        
        # enlève distorsion du pixel
        pix_rec = self.cameraModel.rectifyPoint( (pixD[0],pixD[1]) )
        print(pix_rec)
        
        # donne le vecteur unitaire entre le point et le centre de la caméra
        # (passant par un rayon laser non fixé), dans le ref de l'image
        p_3D = self.cameraModel.projectPixelTo3dRay((pix_rec[0],pix_rec[1]))
        
        print(p_3D)
        
        # passage de ref image à ref camera pour vecteur unitaire
        p_3D = np.matmul(self.R_rpic2rcam,p_3D)
        
        print(p_3D)
        
        # associe un rayon laser au vecteur unitaire grâce à l'information 
        # de profondeur (pix[2])
        Mp3d = np.array([p_3D[0],p_3D[1],p_3D[2]]).T
        
        p_real_3D = (pixD[2]/p_3D[0])*Mp3d

        # # to check if transformation is ok
        #print(p_real_3D)
        
        # take 3d point in camera ref, turn into picture ref
        # p_c_3D = np.matmul(self.R_rcam2rpic,p_real_3D)
        
        # # take 3d point in picture ref in 2D
        # p_check = self.cameraModel.project3dToPixel(p_c_3D)
        
        # print("p_check : ", p_check) 
        
        # # normally, norm(p_check - pixD) < 1e-10
        
        return(p_real_3D)
    
    
    
    
    def __init__(self,rate):
        
    # On cree les variables qui vont stocker les images 
        
        self.CIFrontDepth = None
        self.CIFrontColor = None

    # On cree les subscribers pour chaque image : front, left et right, en couleur et profondeur
    
        rospy.Subscriber("/phantomx/camera_front/depth/camera_info", CamInfoMSG,self.callbackCIFrontDepth)
        rospy.Subscriber("/phantomx/camera_front/color/camera_info", CamInfoMSG,self.callbackCIFrontColor)
        
      

    # On cree les publisher de ces memes images sur des topics differents (avec un rate qu on peut choisir)
    
        self.rate = rospy.Rate(rate)
        
        # self.pubFrontDepth =  rospy.Publisher('/front_depth',Image,queue_size=10)
        # self.pubFrontColor =  rospy.Publisher('/front_color',Image,queue_size=10)
        # self.pubLeftDepth =  rospy.Publisher('/left_depth',Image,queue_size=10)
        # self.pubLeftColor =  rospy.Publisher('/left_color',Image,queue_size=10)
        # self.pubRightDepth =  rospy.Publisher('/right_depth',Image,queue_size=10)
        # self.pubRightColor =  rospy.Publisher('/right_color',Image,queue_size=10)
        
        time.sleep(3)
        
        self.cameraModel = image_geometry.PinholeCameraModel()
        p_pix = PointStamped()
        stamp = rospy.Time()
        
        # on veut : pixel vers 3d
    
        #p_world.header.seq = self.camera_image.header.seq
        #p_world.header.stamp = stamp
        #p_world.header.frame_id = '/world'
        pix = np.array([0,0.0,1.0]).T
        # p_pix.point.x = pix[0]
        # p_pix.point.y = pix[1]
        # p_pix.point.z = pix[2]
        
        
        # listener = tf.TransformListener()
        # listener.waitForTransform('/camera_left', '/base_link', stamp, rospy.Duration(1.0))
        # # transform de camera vers base
        # p_camera = listener.transformPoint('/camera_left', p_world)
        
        self.R_rcam2rpic = np.array([[0.0,-1.0,0.0],[0.,0.,-1.0],[1.0,0.0,0.0]])
        self.R_rpic2rcam = np.linalg.inv(self.R_rcam2rpic)
        
        print(self.R_rpic2rcam)
        
        pr3d = self.fromPix2camRef(pix)
        
        
        
        
        
    
    
    # cas de passage de world (3d) vers pixel (2d)
    
    # passage de réf camera à ref image
    #     # The navigation frame has X pointing forward, Y left and Z up, whereas the
    # # vision frame has X pointing right, Y down and Z forward; hence the need to
    # # reassign axes here.
        
        # (x,y,z) : ref picture
        # p_cam.point : ref camera
        

        
        
        
    #     x = -p_camera.point.y
    #     y = -p_camera.point.z
    #     z = p_camera.point.x
    
        # camera = image_geometry.PinholeCameraModel()
        # camera.fromCameraInfo(camera_info)
    #     p_image = camera.project3dToPixel((x, y, z))
    
            
            
        #     self.publisherGeneral()     
        # # On spin
        #     rospy.spin()
        





if __name__ == "__main__":
    rospy.init_node("crackMapping", anonymous=False, log_level=rospy.DEBUG)
    cm = CrackMap(0.1)
    # scan = rospy.Subscriber("/phantomx/", LaserScan, lidarRead)
    

