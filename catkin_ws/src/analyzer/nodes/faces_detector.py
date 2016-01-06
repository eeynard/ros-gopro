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
    def __init__(self, sendPicturePublisher, fov, vidRes): 
        threading.Thread.__init__(self)
        self.Terminated = False
        self.image = None
        self.oldImage = None
        
        #Image resolutions
        self.vidRes = vidRes
        self.fov = fov
        
        #Find needed resources
        rospack = rospkg.RosPack()
        path = rospack.get_path('analyzer')
        self.face_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(path + '/resources/haarcascade_eye.xml')
        
        self.bridge = CvBridge()
        self.sendPicturePub = sendPicturePublisher
        
    def run(self):
    	while not self.Terminated and not rospy.is_shutdown():
            if self.image is None:
                continue
            if self.image == self.oldImage:
                time.sleep(0.3)
                continue
            self.oldImage = self.image
            try:
                image = self.bridge.imgmsg_to_cv2(self.image)
                self.detectFaces(image)
            except CvBridgeError as e:
                rospy.logerr(e)
            
	
    def stop(self):
        self.Terminated = True
        
    def detectFaces(self, img):
        if img is None:
            return
        rospy.logerr("Analyzing...")
        # Determine vertical video angle
        verticalAngle = self.fov * self.vidRes
        div = float(img.shape[1])/1000.0
        small = cv2.resize(img, (0,0), fx=(1/div), fy=(1/div))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        (height, width, _) = small.shape
        cv2.circle(img, (int(width/2*div), int(height/2*div)), 3, (0, 255, 0), -1)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            rectangle = Rect((x,y,w,h))
            (xCent, yCent) = rectangle.get_center()
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = small[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            if len(eyes) >= 2 :
                cv2.putText(img,"Face detected", (0,150), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 5)
                cv2.circle(img, (int(xCent*div), int(yCent*div)), 3, (0, 255, 0), -1)
                cv2.rectangle(img,(int(x*div),int(y*div)),(int((x+w)*div),int((y+h)*div)),(255,0,0),5)
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
        
        self.sendPicturePub.publish(self.bridge.cv2_to_imgmsg(img, encoding="passthrough"))


