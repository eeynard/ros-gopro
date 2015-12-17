#!/usr/bin/env python

import rospy
from gopro.msg import Status
from sensor_msgs.msg import Image
from gopro_wrapper import GoProWrapper

gopro = None


def talker():
    global gopro

    status_publisher = rospy.Publisher('gopro/status', Status, queue_size=10)
    image_publisher = rospy.Publisher('gopro/camera/image', Image, queue_size=2)
    rospy.init_node('gopro', log_level=rospy.DEBUG)

    rate = rospy.Rate(1)

    rospy.logerr("test")

    while not rospy.is_shutdown():
        image = gopro.image()

        if image:
            image_publisher.publish(image)

        status = gopro.status()
        status_publisher.publish(status)

        rate.sleep()

if __name__ == '__main__':
    try:
        gopro = GoProWrapper('10.5.5.9', 'yeahbabyyeah')
        #gopro.switch(True)

        talker()
    except rospy.ROSInterruptException:
        pass
