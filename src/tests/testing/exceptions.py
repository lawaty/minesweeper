from helpers.Logger import *

class OutOfRangeSignal(BaseCustomException):
  def __init__(self, msg=""):
    super().__init__(msg, "warn")

class PCAConnectionError(BaseCustomException):
  def __init__(self, msg=""):
    super().__init__(msg, "error")
