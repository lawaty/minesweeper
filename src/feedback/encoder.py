#!/usr/bin/env python3

from sensors.HallEncoder import HallEncoder, GPIOConnectionError, InvalidState
from sensors.HoverMotion import HoverMotion
from time import sleep
import rospy
from feedback.msg import Encoder
from helpers.Logger import *

rospy.init_node('encoder', anonymous=True)
encoder_pub = rospy.Publisher('encoder', Encoder, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz
logger = Logger.getInst()

while True:
    try:
        encoder1 = HallEncoder((17, 27, 22))
        encoder2 = HallEncoder((18, 28, 23))
        break
    except InvalidState or GPIOConnectionError as e:
        logger.logE(e)
        logger.log("Retrying in 3 secs")
        sleep(3)

motion_estimator = HoverMotion(encoder1, encoder2)

while not rospy.is_shutdown():
    motion_estimator.update()

    msg = Encoder()
    msg.velocity = encoder.getVelocity()
    msg.disp = encoder.get_disp()
    encoder_pub.publish(msg)
    rate.sleep()