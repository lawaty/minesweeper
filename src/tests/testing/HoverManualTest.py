#!/usr/bin/env python3

import time
from helpers.Logger import *
from testing.interfaces import IHover
from testing.exceptions import OutOfRangeSignal

class HoverManualTest:
  SAFE_REGION = 0.9

  __slots__ = ['__hover', '__logger']
  def __init__(self, hover: IHover = None):
    self.__hover = hover
    self.__logger = Logger.getInst()

  def stop(self):
    self.__hover.apply(self.__neutral, self.__neutral)
    self.log("Stopped")
    self.wait(3)

  def moveForward(self, max_speed = False):
    self.__hover.apply(trans = self.__max_pos if max_speed else self.__normal_pos)

    self.log(f"Moving Forward with " + "max speed" if max_speed else "average speed")
    self.wait(5)

  def moveBackward(self, max_speed = False):
    self.__hover.apply(self.__neutral, self.__max_neg if max_speed else self.__normal_neg)

    self.log(f"Moving Backward with " + "max speed" if max_speed else "average speed")
    self.wait(5)

  def rotateCW(self, max_speed = False):
    self.__hover.apply(rot=self.__max_pos if max_speed else self.__normal_pos)
    self.log(f"Rotating Clockwise with " + "max speed" if max_speed else "average speed")
    self.wait(5)

  def rotateACW(self, max_speed = False):
    self.__hover.apply(rot=self.__max_neg if max_speed else self.__normal_neg)
    self.log(f"Rotating Anti-Clickwise with " + "max speed" if max_speed else "average speed")
    self.wait(5)

  def wait(self, secs:int):
    self.log(f"Waiting {secs} secs")
    time.sleep(secs)

  def log(self, msg):
    self.__logger.log(msg)

  def run(self):
    try:
      self.__hover.apply(10000000, 1000000)
      self.log("Invalid Range Exception Not Thrown")
    except OutOfRangeSignal:
      pass

    self.stop()
    self.wait(3)

    self.log("Testing Simple Translation")
    self.moveForward(max_speed=False)
    self.wait(5)
    self.moveForward(max_speed=True)
    self.wait(5)
    self.moveBackward(max_speed=False)
    self.wait(5)
    self.moveBackward(max_speed=True)
    self.wait(5)

    self.log("Testing Simple Rotation")
    self.rotateCW(max_speed=False)
    self.wait(5)
    self.rotateCW(max_speed=True)
    self.wait(5)
    self.rotateACW(max_speed=False)
    self.wait(5)
    self.rotateACW(max_speed=True)
    self.wait(5)

    self.log("Testing Composite Movement (Translational and Rotational at the same time)")

    self.moveForward()
    self.rotateCW()
    self.wait(7)

    self.moveForward()
    self.rotateACW()
    self.wait(7)

    self.moveBackward()
    self.rotateCW()
    self.wait(7)

    self.moveBackward()
    self.rotateACW()
    self.wait(7)

## Example Run
# rospy.init_node("hover_manual_test", anonymous=True)

# while True:
#   try:
#     hover = Hover((12, 13), die_on_range_error=True)
#     self.log("Hover Initialization Succeeded")
#     break
#   except PCAConnectionError as e:
#     self.log("Retrying in 3 secs")
#     time.sleep(3)

# test = HoverManualTest(hover)
# test.run()
