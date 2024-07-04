from sensors.HallEncoder import HallEncoder, InvalidState, InvalidTransition
from helpers.Logger import *


class HoverMotion:
  """
  Class estimates motion characteristics for a hover of two motors
  """
  __slots__ = ['__encoder1', '__encoder2']
  def __init__(self, encoder1: HallEncoder, encoder2: HallEncoder):
    self.__encoder1 = encoder1
    self.__encoder2 = encoder2
    self.__logger = Logger.getInst()

    self.__velocity_buffer = []

  def update(self):
    try:
      self.__encoder1.update()
    except InvalidState or InvalidTransition as e:
      self.__logger.logE(e)
      self.__logger.log("Skipped this reading. This may cause a whole revolution to be ignored")

    try:
      self.__encoder2.update()
    except InvalidState or InvalidTransition as e:
      self.__logger.logE(e)
      self.__logger.log("Skipped this reading. This may cause a whole revolution to be ignored")

    self.__v1 = self.__encoder1.getVelocity()
    self.__v2 = self.__encoder2.getVelocity()