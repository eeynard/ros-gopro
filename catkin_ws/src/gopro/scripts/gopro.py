#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gopro.msg import Status
from gopro_wrapper import GoProWrapper

gopro = None


def talker():
    global gopro

    pub = rospy.Publisher('gopro/status', Status, queue_size=10)
    rospy.init_node('gopro')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(gopro.status())
        rate.sleep()

if __name__ == '__main__':
    try:
        gopro = GoProWrapper('10.5.5.9', 'yeahbabyyeah')
        rospy.loginfo(gopro.switch(True))

        talker()
    except rospy.ROSInterruptException:
        pass
