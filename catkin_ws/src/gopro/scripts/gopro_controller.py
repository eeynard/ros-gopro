#!/usr/bin/env python

import rospy
from gopro.msg import Status
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
from gopro_wrapper import GoProWrapper

gopro = None
take_picture_publisher = None
picture_publisher = None


def take_picture_callback(data):
    if data.data:
        picture = gopro.picture()
        #picture = gopro.picture_from_preview()

        if picture:
            picture_publisher.publish(picture)


def talker():
    global gopro
    global take_picture_publisher
    global picture_publisher

    take_picture_publisher = rospy.Publisher('gopro/camera/take_picture', Int64, queue_size=1)
    status_publisher = rospy.Publisher('gopro/status', Status, queue_size=10)
    picture_publisher = rospy.Publisher('gopro/camera/picture', Image, queue_size=2)
    rospy.init_node('gopro', log_level=rospy.DEBUG)

    rospy.Subscriber('gopro/camera/take_picture', Int64, take_picture_callback)

    rate = rospy.Rate(1)

    gopro.start_preview()

    keep_alive = 1

    while not rospy.is_shutdown():
        if keep_alive:
            gopro.keep_alive_preview()
            keep_alive = 0
        else:
            keep_alive = 1

        status = gopro.status()

        if status:
            status_publisher.publish(status)

        rate.sleep()

if __name__ == '__main__':
    try:
        gopro = GoProWrapper('10.5.5.9', 'yeahbabyyeah')

        talker()
    except rospy.ROSInterruptException:
        pass
