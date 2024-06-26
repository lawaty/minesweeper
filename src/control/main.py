#!/usr/bin/env python3

import rospy
from control.msg import motors
from actuators.Hover import Hover  # Example import from actuators package
from tests.HoverManualTest import HoverManualTest

# Your main script logic follows...
def main():
    rospy.init_node("my_node", anonymous=True)
    # Use imported classes or functions here

if __name__ == "__main__":
    main()