#!/usr/bin/env python3

from actuators.Hover import Hover, PCAConnectionError
import rospy
import time

class HoverManualTest:
  SAFE_REGION = 0.9
  def __init__(self, hover: Hover = None):
    self.__hover = hover

  def setHover(self, hover: Hover):
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

rospy.init_node("hover_manual_test", anonymous=True)

test = HoverManualTest()

while True:
  try:
    hover = Hover(12, 13)
    test.log("Hover Initialization Succeeded")
    break
  except PCAConnectionError:
    test.log("Couldn't initialize PCA Driver, Retrying in 3 secs")
    time.sleep(3)

test.stop()
test.wait(3)

test.log("Testing Simple Translation")
test.moveForward(max_speed=False)
test.wait(5)
test.moveForward(max_speed=True)
test.wait(5)
test.moveBackward(max_speed=False)
test.wait(5)
test.moveBackward(max_speed=True)
test.wait(5)

test.log("Testing Simple Rotation")
test.rotateCW(max_speed=False)
test.wait(5)
test.rotateCW(max_speed=True)
test.wait(5)
test.rotateACW(max_speed=False)
test.wait(5)
test.rotateACW(max_speed=True)
test.wait(5)

test.log("Testing Composite Movement (Translational and Rotational at the same time)")

test.moveForward()
test.rotateCW()
test.wait(7)

test.moveForward()
test.rotateACW()
test.wait(7)

test.moveBackward()
test.rotateCW()
test.wait(7)

test.moveBackward()
test.rotateACW()
test.wait(7)