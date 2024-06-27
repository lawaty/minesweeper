#!/usr/bin/env python3

from sensors.HallEncoder import HallEncoder, InvalidState, InvalidTransition, GPIOConnectionError
from time import sleep
import rospy
from feedback.msg import Encoder
from helpers.Logger import *

rospy.init_node('encoder_publisher', anonymous=True)
encoder_pub = rospy.Publisher('encoder', Encoder, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz
logger = Logger.getInst()

while True:
    try:
        encoder = HallEncoder((17, 27, 22))  # Example GPIO pins
        break
    except InvalidState or GPIOConnectionError as e:
        logger.logE(e)
        logger.log("Retrying in 3 secs")
        sleep(3)

while not rospy.is_shutdown():
    try:
        encoder.update()
    except InvalidState or InvalidTransition as e:
        logger.logE(e)
        logger.log("Skipped this reading. This may cause a whole revolution to be ignored")

    msg = Encoder()
    msg.velocity = encoder.get_velocity()
    msg.disp = encoder.get_disp()
    encoder_pub.publish(msg)
    rate.sleep()