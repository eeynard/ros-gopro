#!/usr/bin/env python

import rospy
from gopro_analyzer.msg import FacePosition
from std_msgs.msg import Int16

gvangle = 0
ghangle = 0
haveReceived = False

def talker(received):
    global gvangle
    global ghangle
    global haveReceived
    gvangle = received.vertical
    ghangle = received.horizontal
    haveReceived = True

def position_handler(data):
    rospy.loginfo(rospy.get_caller_id() + "I received %s", data)
    talker(data)

def listener():
    global haveReceived
    rospy.Subscriber("/analyzer/face/position", FacePosition, position_handler)

    pitchPub = rospy.Publisher('/gimbal/pitch_control', Int16, queue_size=10)
    headingPub = rospy.Publisher('/gimbal/heading_control', Int16, queue_size=10)

    rate = rospy.Rate(10)
    while True:
        if haveReceived:
            pitchPub.publish(int(gvangle))
            headingPub.publish(int(ghangle))
            haveReceived = False

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gimbal_controller', anonymous=True)
    listener()
