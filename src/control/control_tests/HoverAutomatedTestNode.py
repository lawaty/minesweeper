#!/usr/bin/env python3

import rospy
from control_tests.adapters import HoverAutomatedAdapter
from testing.HoverAutomatedTest import HoverAutomatedTest
from drivers.PWM import PCAConnectionError
import time

while True:
  try:
    hover = HoverAutomatedAdapter((11, 12), True)
    break
  except PCAConnectionError as e:
    rospy.loginfo("Retrying in 3 secs")
    time.sleep(3)

test = HoverAutomatedTest(hover)
test.run()