#!/usr/bin/env python3

import board
import busio
from adafruit_pca9685 import PCA9685

class PWM:
  __slots__ = ['__pca', '__pin', '__min', '__max']
  def __init__(self, pin:int, config:tuple):
    self.__pin = pin
    self.__pca = PCA.getInst()
    if len(config) != 2:
      raise ValueError("PWM config need to be a tuple of two values, min and max")
    
    self.__min = config[0]
    self.__max = config[1]

  def apply(self, val:int):
    self.__pca.channels[self.__pin].duty_cycle = self.__map(val)

  def __map(self, val:int) -> int:
    """
    @throws OutOfRangeSignal
    """
    if val < self.__min or val > self.__max:
      raise OutOfRangeSignal(f"PWM Value must be between {self.__min} and {self.__max} but found {val}")

    return (val - self.__min) / (self.__max - self.__min) * 100 * self.__resolution
  
class PCA(PCA9685):
  @staticmethod
  def getInst() -> PCA: 
    if not PCA.__inst:
      i2c = busio.I2C(board.SCL, board.SDA)
      PCA.__inst = PCA(i2c)
    
    return PCA.__inst

class OutOfRangeSignal(Exception):
  pass