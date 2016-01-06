#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int64
from gopro.msg import Status
from faces_detector import FacesDetector

class Analyzer:
    def callbackGoProPicture(self, data):
        rospy.logerr('callbackGoProPicture')
        self.facesDetector.image = data
        self.cameraInfoPub.publish(CameraInfo())
        self.takePicturePub.publish(1)
            
    def callbackGoProStatus(self, data):
        rospy.logerr('callbackGoProStatus')
        cameraSX = data.sx
        if cameraSX :
            if cameraSX.vidres in self.videoResolutions:
                self.facesDetector.vidRes = self.videoResolutions[cameraSX.vidres]
            self.facesDetector.fov = float(cameraSX.fov)
            self.statusSub.unregister()
            

    def __init__(self):
        #GoPro video resolutions
        self.videoResolutions = {'720p SuperView' : (9.0/16.0), '920p' : (3.0/4.0)}
        rospy.init_node('analyzer', anonymous=True)
        self.initPublish()
        self.facesDetector = FacesDetector(self.sendPicturePub, 170.0, self.videoResolutions['720p SuperView'])
        self.initSubscribe()

    def initSubscribe(self):
        rospy.Subscriber('/gopro/camera/picture', Image, self.callbackGoProPicture)
        self.statusSub = rospy.Subscriber('/gopro/status', Status, self.callbackGoProStatus)

    def initPublish(self):
        self.takePicturePub = rospy.Publisher('/gopro/camera/take_picture', Int64, queue_size=10)
        self.sendPicturePub = rospy.Publisher('/analyzer/video', Image, queue_size=2)
        self.cameraInfoPub = rospy.Publisher('/analyzer/camera_info', CameraInfo, queue_size=10)

    def start(self):
        self.facesDetector.start()
        rospy.spin()

if __name__ == '__main__':
    anal = Analyzer()
    anal.start()

