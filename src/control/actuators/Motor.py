from drivers.PWM import PWM, OutOfRangeSignal

class Motor():
  """
  Class Controls a single hoverbaord motor Separately
  
  """
  MIN = 0
  MAX = 4095

  __slots__ = ['pin', '__pwm']
  def __init__(self, pin):
    self.pin = pin
    self.__pwm = PWM(pin, (Motor.MIN, Motor.MAX))

  def apply(self, speed:int):
    try:
      self.__pwm.apply(speed)
    except OutOfRangeSignal as e:
      pass