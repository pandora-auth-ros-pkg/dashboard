#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
import random

from geometry_msgs.msg import PoseStamped
from pandora_data_fusion_msgs.msg import QrInfo


def publish_qr_alert(delay):
    rospy.init_node('qr_notification')
    pub = rospy.Publisher('/data_fusion/alert_handler/qr_notification', QrInfo)
    msg = QrInfo()

    while not rospy.is_shutdown():
        print('Sending QR alert...')
        msg.id = random.randint(0, 10)
        msg.qrFrameId = 'kinect_frame'
        msg.timeFound = rospy.Time.now()
        msg.qrPose = PoseStamped()
        msg.probability = random.random()
        msg.content = 'qr content'

        pub.publish(msg)
        time.sleep(delay)


if __name__ == '__main__':

    if len(sys.argv) > 1:
        publish_qr_alert(sys.argv[1])
    else:
        publish_qr_alert(7)
