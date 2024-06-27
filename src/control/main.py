#!/usr/bin/env python3

import rospy
from control.msg import motors
from actuators.Hover import Hover

def callback(msg):
    hover.apply(msg.steer, msg.speed)

rospy.init_node("move", anonymous=True)

hover = Hover(1, 2)

rospy.Subscriber("motors", motors, callback)

# Keep the node running
rospy.spin()
