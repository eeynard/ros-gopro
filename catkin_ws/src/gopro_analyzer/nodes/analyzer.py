#!/usr/bin/env python

import threading
import rospy
import cv2
import time
import math
import numpy as np
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from gopro_analyzer.msg import FacePosition
from rect.rect import Rect
  
class Analyzer:
    'Initialisation'
    def __init__(self):
        # Image resolutions
        self.vidRes = None
        self.verticalAngle = None
        self.horizontalAngle = None

        # Init node
        rospy.init_node('analyzer', log_level=rospy.DEBUG)
        
        # Find needed resources
        rospack = rospkg.RosPack()
        path = rospack.get_path('gopro_analyzer')
        self.face_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_eye.xml')

        # Bridge for ROS Image to OpenCV Image
        self.bridge = CvBridge()

        self.initSubscribe()
        self.initPublish()

    def initSubscribe(self):
        rospy.Subscriber('/analyzer/picture/h_angle', Float64, self.callbackPictureHAngle)
        rospy.Subscriber('/analyzer/picture/vidRes', Float64, self.callbackPictureVidRes)
        rospy.Subscriber('/analyzer/picture/raw', Image, self.callbackPictureRaw)

    'Initialisation of topic publications'
    def initPublish(self):
        self.verticalAnglePub = rospy.Publisher('/analyzer/picture/v_angle', Float64, queue_size=10)
        self.facePositionPub = rospy.Publisher('/analyzer/face/position', FacePosition, queue_size=10)
        self.analyzedPicturePub = rospy.Publisher('/analyzer/picture/analyzed', Image, queue_size=2)
        self.faceDistancePub = rospy.Publisher('/analyzer/face/distance', Float64, queue_size=2)
    
    'Detect faces'
    def detectFaces(self, img, verticalAngle, horizontalAngle):
        rospy.logdebug('detectFaces')
        # Define a divider to resize picture
        div = float(img.shape[1])/1000.0

        # Resize picture
        small = cv2.resize(img, (0,0), fx=(1/div), fy=(1/div))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        (height, width, _) = small.shape

        # Draw center point on unresized picture
        cv2.circle(img, (int(width/2*div), int(height/2*div)), 3, (0, 255, 0), -1)

        # Detect faces in picture
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        # Analyze each face
        for (x,y,w,h) in faces:
            cv2.putText(img,"Face detected", (0,150), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
            rectangle = Rect((x,y,w,h))
            
            # Get face center
            (xCent, yCent) = rectangle.get_center()
            
            # Draw face center and face rectangle
            cv2.circle(img, (int(xCent*div), int(yCent*div)), 3, (0, 255, 0), -1)
            cv2.rectangle(img,(int(x*div),int(y*div)),(int((x+w)*div),int((y+h)*div)),(255,0,0),5)
            
            # Try to see if there are eyes
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = small[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            
            # Compute face distance
            distance = (w*(-0.83)+197.7)
            
            if len(eyes) == 2 :
                cv2.putText(img,"Eyes detected", (0,450), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
                
                #Compute distance between eyes
                eye1 = Rect(eyes[0])
                eye2 = Rect(eyes[1])
                pixDistance = math.fabs(eye1.get_center()[0] - eye2.get_center()[0])
                rospy.logerr('---------------' + str(pixDistance))
                distance = (pixDistance*(-0.27)+63.8)
                
            cv2.putText(img,"Distance : " + str(distance), (0,300), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
            
            #Compute distance between screen center and face center in degree
            horizontalDistanceAngle = 0
            verticalDistanceAngle = 0
            facePosition = FacePosition()
            if xCent < (width/2.0):
                distance = math.fabs(xCent - (width/2.0))
                horizontalDistanceAngle = self.horizontalAngle * distance / width
                facePosition.horizontal = - horizontalDistanceAngle
                rospy.logdebug("Left : " + str(horizontalDistanceAngle) + " degrees")
            elif xCent > (width/2.0):
                distance = math.fabs(xCent - (width/2.0))
                horizontalDistanceAngle = self.horizontalAngle * distance / width
                facePosition.horizontal = horizontalDistanceAngle
                rospy.logdebug("Right : " + str(horizontalDistanceAngle) + " degrees")
            if yCent < (height/2.0):
                distance = math.fabs(yCent - (height/2.0))
                verticalDistanceAngle = verticalAngle * distance / height
                facePosition.vertical = verticalDistanceAngle
                rospy.logdebug("Top : " + str(verticalDistanceAngle) + " degrees")
            elif yCent > (height/2.0):
                distance = math.fabs(yCent - (height/2.0))
                verticalDistanceAngle = verticalAngle * distance / height
                facePosition.vertical = - verticalDistanceAngle
                rospy.logdebug("Bottom : " + str(verticalDistanceAngle) + " degrees")
            # Publish face position
            self.facePositionPub.publish(facePosition)
                
        # Publish analyzed picture     
        self.analyzedPicturePub.publish(self.bridge.cv2_to_imgmsg(img, encoding="passthrough"))
        

    def callbackPictureHAngle(self, data):
        self.horizontalAngle = data.data

    def callbackPictureVidRes(self, data):
        self.vidRes = data.data

    def callbackPictureRaw(self, data):
        if self.horizontalAngle is None or self.vidRes is None:
            return
        self.computeVerticalAngle(self.horizontalAngle, self.vidRes)
        try:
            # Convert image from ROS to OpenCV
            image = self.bridge.imgmsg_to_cv2(data)
            self.detectFaces(image, self.verticalAngle, self.horizontalAngle)
        except CvBridgeError as e:
            rospy.logerr(e)

    'Determine vertical video angle'
    def computeVerticalAngle(self, horizontalAngle, vidRes):
        self.verticalAngle = horizontalAngle * (1.0/vidRes)
        self.verticalAnglePub.publish(self.verticalAngle)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    anal = Analyzer()
    anal.start()
