#!/usr/bin/env python3

import rospy
import time
from helpers.Logger import *
from testing.interfaces import IHover
from testing.exceptions import OutOfRangeSignal


class HoverAutomatedTest:
	SAFE_REGION = 0.9

	__slots__ = ["__hover", "__logger", "__steer", "__speed"]

	def __init__(self, hover: IHover = None):
		self.__hover = hover
		self.__logger = Logger.getInst()

		self.__steer = None
		self.__speed = None

		self.__subscriber = rospy.Subscriber("encoder")

	def setHover(self, hover: IHover):
		self.__hover = hover

	def __apply(self, rot: int = None, trans: int = None):
		if rot is not None:
			self.__steer = rot
		if trans is not None:
			self.__speed = trans
			
		self.__apply(rot, trans)

	def stop(self):
		self.__apply(self.__neutral, self.__neutral)
		self.log("Stopped")
		self.wait(3)

	def moveForward(self, max_speed=False):
		self.__apply(trans=self.__max_pos if max_speed else self.__normal_pos)

		self.log(
			f"Moving Forward with " + "max speed" if max_speed else "average speed"
		)
		self.wait(5)

	def moveBackward(self, max_speed=False):
		self.__apply(
			self.__neutral, self.__max_neg if max_speed else self.__normal_neg
		)

		self.log(
			f"Moving Backward with " + "max speed" if max_speed else "average speed"
		)
		self.wait(5)

	def rotateCW(self, max_speed=False):
		self.__apply(rot=self.__max_pos if max_speed else self.__normal_pos)

		self.log(
			f"Rotating Clockwise with " + "max speed" if max_speed else "average speed"
		)
		self.wait(5)

	def rotateACW(self, max_speed=False):
		self.__apply(rot=self.__max_neg if max_speed else self.__normal_neg)

		self.log(
			f"Rotating Anti-Clickwise with " + "max speed"
			if max_speed
			else "average speed"
		)
		self.wait(5)

	def wait(self, secs: int):
		self.log(f"Waiting {secs} secs")
		time.sleep(secs)

	def log(self, msg):
		self.__logger.log(msg)

	def verify(self):
		self.__logger.log("Verifying ...")
		time.sleep(3)
		self.__

		pass

	def run(self):
		try:
			self.__apply(10000000, 1000000)
			self.log("Invalid Range Exception Not Thrown")
		except OutOfRangeSignal:
			pass

		self.stop()
		self.verify()

		self.log("Testing Simple Translation")
		self.moveForward(max_speed=False)
		self.verify()
		self.moveForward(max_speed=True)
		self.verify()
		self.moveBackward(max_speed=False)
		self.verify()
		self.moveBackward(max_speed=True)
		self.verify()

		self.log("Testing Simple Rotation")
		self.rotateCW(max_speed=False)
		self.verify()
		self.rotateCW(max_speed=True)
		self.verify()
		self.rotateACW(max_speed=False)
		self.verify()
		self.rotateACW(max_speed=True)
		self.verify()

		self.log(
			"Testing Composite Movement (Translational and Rotational at the same time)"
		)

		self.moveForward()
		self.rotateCW()
		self.verify()

		self.moveForward()
		self.rotateACW()
		self.verify()

		self.moveBackward()
		self.rotateCW()
		self.verify()

		self.moveBackward()
		self.rotateACW()
		self.verify()
