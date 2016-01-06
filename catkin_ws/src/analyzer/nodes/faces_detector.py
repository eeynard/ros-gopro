import threading
import rospy
import cv2
import time
import math
import numpy as np
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from rect.rect import Rect
  
class FacesDetector(threading.Thread):
    'Initialisation : sendPicturePublisher(rospy.Publisher) = publisher, fov(float) = horizontal camera angle, vidRes(float) = Video resolution(16/9, 3/4)'
    def __init__(self, sendPicturePublisher, fov, vidRes): 
        threading.Thread.__init__(self)
        self.image = None
        self.oldImage = None
        
        # Image resolutions
        self.vidRes = vidRes
        self.fov = fov
        
        # Find needed resources
        rospack = rospkg.RosPack()
        path = rospack.get_path('analyzer')
        self.face_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_eye.xml')

        # Bridge for ROS Image to OpenCV Image
        self.bridge = CvBridge()

        # Publisher for analized pictures
        self.sendPicturePub = sendPicturePublisher

    'Thread runner'
    def run(self):
    	while not rospy.is_shutdown():
            # If no image, do nothing
            if self.image is None:
                continue
            # If image same as precedent, do nothing
            if self.image == self.oldImage:
                time.sleep(0.3)
                continue
            self.oldImage = self.image
            try:
                # Convert image from ROS to OpenCV
                image = self.bridge.imgmsg_to_cv2(self.image)
                self.detectFaces(image)
            except CvBridgeError as e:
                rospy.logerr(e)
    
    'Detect faces'
    def detectFaces(self, img):
        if img is None:
            return
        # Determine vertical video angle
        verticalAngle = self.fov * self.vidRes

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
            cv2.putText(img,"Face detected ", (0,150), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
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
            
            if len(eyes) >= 2 :
                cv2.putText(img,"Eyes detected ", (0,300), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
            
            #Compute distance between screen center and face center in degree
            horizontalDistanceAngle = 0
            verticalDistanceAngle = 0
            if xCent < (width/2.0):
                distance = math.fabs(xCent - (width/2.0))
                horizontalDistanceAngle = self.fov * distance / width
                rospy.logerr("Left : " + str(horizontalDistanceAngle) + " degrees")
            elif xCent > (width/2.0):
                distance = math.fabs(xCent - (width/2.0))
                horizontalDistanceAngle = self.fov * distance / width
                rospy.logerr("Right : " + str(horizontalDistanceAngle) + " degrees")
            if yCent < (height/2.0):
                distance = math.fabs(yCent - (height/2.0))
                verticalDistanceAngle = verticalAngle * distance / height
                rospy.logerr("Top : " + str(verticalDistanceAngle) + " degrees")
            elif yCent > (height/2.0):
                distance = math.fabs(yCent - (height/2.0))
                verticalDistanceAngle = verticalAngle * distance / height
                rospy.logerr("Bottom : " + str(verticalDistanceAngle) + " degrees")
                
        # Publish analyzed picture        
        self.sendPicturePub.publish(self.bridge.cv2_to_imgmsg(img, encoding="passthrough"))


