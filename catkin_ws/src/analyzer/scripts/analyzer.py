import numpy as np
import cv2
import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rect.rect import Rect

class Analyzer:
	def __init__(self):
		self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		self.eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
		self.bridge = CvBridge()
		
		self.cameraHorizontalAngles = {'wide' : 170, 'normal' : 127, 'narrow' : 90}
		self.photoFormat = {'16:9' : (9.0/16.0), '4:3' : (3.0/4.0)}

	def callbackImage(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		
	def listener(self):
		rospy.init_node('AnalyzerGoPro', anonymous=True)

		rospy.Subscriber("FluxGoPro", Image, callbackImage)

		rospy.spin()
	
	def test(self, horizontalAngleName, photoFormatName):
		horizontalAngle = 0
		if horizontalAngleName not in self.cameraHorizontalAngles:
			return
		else:
			horizontalAngle = self.cameraHorizontalAngles[horizontalAngleName]
		
		photoFormat = 1;
		if photoFormatName not in self.photoFormat:
			return
		else :
			photoFormat = self.photoFormat[photoFormatName]
		
		verticalAngle = horizontalAngle * photoFormat

		img = cv2.imread('test.jpg')
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		(height, width, _) = img.shape
		cv2.circle(img, (int(width/2), int(height/2)), 3, (0, 0, 255), -1)
		faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
		for (x,y,w,h) in faces:
			rectangle = Rect((x,y,w,h))
			(xCent, yCent) = rectangle.get_center()

			
			cv2.circle(img, (int(xCent), int(yCent)), 3, (0, 255, 0), -1)
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			
			roi_gray = gray[y:y+h, x:x+w]
			roi_color = img[y:y+h, x:x+w]
			eyes = self.eye_cascade.detectMultiScale(roi_gray)
			
			if len(eyes) >= 2 :
				cv2.putText(img,"Face detected", (0,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255))
				
				#Compute distance between screen center and face center in degree
				horizontalDistanceAngle = 0
				verticalDistanceAngle = 0
				if xCent < (width/2.0):
					distance = math.fabs(xCent - (width/2.0))
					horizontalDistanceAngle = horizontalAngle * distance / width
					print "Left : " + str(horizontalDistanceAngle) + "degrees"
				elif xCent > (width/2.0):
					distance = math.fabs(xCent - (width/2.0))
					horizontalDistanceAngle = horizontalAngle * distance / width
					print "Right : " + str(horizontalDistanceAngle) + "degrees"
				if yCent < (height/2.0):
					distance = math.fabs(yCent - (height/2.0))
					verticalDistanceAngle = verticalAngle * distance / height
					print "Top : " + str(verticalDistanceAngle) + "degrees"
				elif yCent > (height/2.0):
					distance = math.fabs(yCent - (height/2.0))
					verticalDistanceAngle = verticalAngle * distance / height
					print "Bottom : " + str(verticalDistanceAngle) + "degrees"

		cv2.imshow('img',img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

if __name__ == '__main__':
    anal = Analyzer()
    anal.test('narrow', '4:3')

