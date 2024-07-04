#!/usr/bin/env python3

import rospy
from control_tests.adapters import HoverAdapter
from testing.HoverManualTest import HoverManualTest
from drivers.PWM import PCAConnectionError
import time

while True:
  try:
    hover = HoverAdapter((11, 12), True)
    break
  except PCAConnectionError as e:
    rospy.loginfo("Retrying in 3 secs")
    time.sleep(3)

test = HoverManualTest(hover)
test.run()