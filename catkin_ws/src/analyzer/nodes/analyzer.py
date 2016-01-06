#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int64
from gopro.msg import Status
from faces_detector import FacesDetector

class Analyzer:
    'Callback for /gopro/camera/picture topic'
    def callbackGoProPicture(self, data):
        # Get image from GoPro and give it to FacesDetector
        self.facesDetector.image = data
        # Publish CameraInfo for RVIZ Camera (do not work)
        self.cameraInfoPub.publish(CameraInfo())
        # Ask new picture
        self.takePicturePub.publish(1)

    'Callback for /gopro/status topic'
    def callbackGoProStatus(self, data):
        cameraSX = data.sx
        if cameraSX :
            # Get Video Resolution
            if cameraSX.vidres in self.videoResolutions:
                self.facesDetector.vidRes = self.videoResolutions[cameraSX.vidres]
            # Get Horizontal Camera Angle
            self.facesDetector.fov = float(cameraSX.fov)
            self.statusSub.unregister()
            
    'Init'
    def __init__(self):
        # GoPro video resolutions 
        self.videoResolutions = {'720p SuperView' : (9.0/16.0), '920p' : (3.0/4.0), 'WVGA' : (9.0/16.0)}
        rospy.init_node('analyzer', anonymous=True)
        self.initPublish()
        self.facesDetector = FacesDetector(self.analyzePub, 170.0, self.videoResolutions['720p SuperView'])
        self.initSubscribe()
        self.takePicturePub.publish(1)

    'Initialisation of topic subscriptions'
    def initSubscribe(self):
        rospy.Subscriber('/gopro/camera/picture', Image, self.callbackGoProPicture)
        self.statusSub = rospy.Subscriber('/gopro/status', Status, self.callbackGoProStatus)

    'Initialisation of topic publications'
    def initPublish(self):
        self.takePicturePub = rospy.Publisher('/gopro/camera/take_picture', Int64, queue_size=10)
        self.analyzePub = rospy.Publisher('/analyzer/analyze', Image, queue_size=2)
        self.cameraInfoPub = rospy.Publisher('/analyzer/camera_info', CameraInfo, queue_size=10)

    'Launch analysis'
    def start(self):
        self.facesDetector.start()
        rospy.spin()

if __name__ == '__main__':
    anal = Analyzer()
    anal.start()

