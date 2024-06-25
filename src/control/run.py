#!/usr/bin/env python3

import rospy
from control.msg import motors


def motor_value_callback(data):
    rospy.loginfo(f"Received motor values: motor1={data.steer}, motor2={data.speed}")


def motor_listener():
    rospy.init_node("motor_listener_node", anonymous=True)
    rospy.Subscriber("/motor_values", motors, motor_value_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        motor_listener()
    except rospy.ROSInterruptException:
        pass
