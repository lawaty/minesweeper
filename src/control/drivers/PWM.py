#!/usr/bin/env python3

import board
import busio
from adafruit_pca9685 import PCA9685
from helpers.Logger import *

class OutOfRangeSignal(BaseCustomException):
  def __init__(self, msg=""):
    super().__init__(msg, "warn")

class PCAConnectionError(BaseCustomException):
  def __init__(self, msg=""):
    super().__init__(msg, "error")

class PWM:
  __slots__ = ['__pca', '__pin', '__min', '__max', '__logger']

  @throws(ValueError, PCAConnectionError)
  def __init__(self, pin:int, config:tuple):
    self.__pin = pin
    self.__logger = Logger.getInst()

    if len(config) != 2:
      raise ValueError("PWM config need to be a tuple of two values, min and max")
    
    self.__pca = PCA.getInst()
    
    self.__min = config[0]
    self.__max = config[1]

  def apply(self, val:int):
    """
    PWM Class automatically maps val to duty cycle (ranges from 0 to 4095)
    """
    self.__pca.channels[self.__pin].duty_cycle = self.__map(val)

  def __map(self, val:int) -> int:
    if val < self.__min or val > self.__max:
      self.__logger.logE(OutOfRangeSignal(f"PWM Value must be between {self.__min} and {self.__max} but found {val}"))

    val = max(self.__min, val)
    val = min(self.__max, val)
    return int((val - self.__min) / (self.__max - self.__min) * PCA.RESOLUTION)
  
class PCA(PCA9685):
  RESOLUTION = 4095
  __inst = None

  @staticmethod
  def getInst(): 
    if PCA.__inst is None:
      try:
        i2c = busio.I2C(board.SCL, board.SDA)
        PCA.__inst = PCA(i2c)
      except Exception as e:
        raise PCAConnectionError()
    
    return PCA.__inst