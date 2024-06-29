#!/usr/bin/env python3

import time
from helpers.Logger import *
from testing.interfaces import IHoverAutomated
from testing.exceptions import OutOfRangeSignal

class HoverAutomatedTest:
  SAFE_REGION = 0.9

  __slots__ = ['__hover', '__logger']
  def __init__(self, hover: IHoverAutomated = None):
    self.__hover = hover

  def setHover(self, hover: IHoverAutomated):
    self.__hover = hover

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
    """
    :todo make use of logger package
    """
    rospy.loginfo(msg)

  def run(self):
    try:
      self.__hover.apply(10000000, 1000000)
      self.log("Invalid Range Exception Not Thrown")
    except OutOfRangeSignal:
      pass

    self.stop()
    self.__hover.verify()

    self.log("Testing Simple Translation")
    self.moveForward(max_speed=False)
    self.__hover.verify()
    self.moveForward(max_speed=True)
    self.__hover.verify()
    self.moveBackward(max_speed=False)
    self.__hover.verify()
    self.moveBackward(max_speed=True)
    self.__hover.verify()

    self.log("Testing Simple Rotation")
    self.rotateCW(max_speed=False)
    self.__hover.verify()
    self.rotateCW(max_speed=True)
    self.__hover.verify()
    self.rotateACW(max_speed=False)
    self.__hover.verify()
    self.rotateACW(max_speed=True)
    self.__hover.verify()

    self.log("Testing Composite Movement (Translational and Rotational at the same time)")

    self.moveForward()
    self.rotateCW()
    self.__hover.verify()

    self.moveForward()
    self.rotateACW()
    self.__hover.verify()

    self.moveBackward()
    self.rotateCW()
    self.__hover.verify()

    self.moveBackward()
    self.rotateACW()
    self.__hover.verify()