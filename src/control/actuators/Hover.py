from drivers.PWM import PWM, OutOfRangeSignal, PCAConnectionError

class Hover():
  """
  Class Controls two motors of a hoverboard together
  No smoothing because it depends on hover stm32 smoothing
  """
  MAX_NEG = 0
  MAX_POS = 4096
  SAFE_REGION = 0.9

  def __init__(self, pin1, pin2):
    self.pin1 = pin1
    self.pin2 = pin2
    self.__rotational = PWM(pin1, (Hover.MAX_NEG, Hover.MAX_POS))
    self.__translational = PWM(pin2, (Hover.MAX_NEG, Hover.MAX_POS))

    self.neutral =  (Hover.MAX_POS + Hover.MAX_NEG) / 2
    self.amplitude = abs(Hover.MAX_POS - self.neutral) * self.SAFE_REGION

    self.__steer = self.__speed = self.neutral

  def apply(self, rot:int, trans:int):
    """
    :throws OutOfRangeSignal
    """
    if not steer:
      steer = self.__steer

    if not speed:
      speed = self.__speed

    self.__rotational.apply(steer)
    self.__translational.apply(speed)

    self.__steer = steer
    self.__speed = speed