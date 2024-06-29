from drivers.PWM import PWM, PCAConnectionError
from helpers.Logger import throws

class Hover:
	"""
	Class Controls two motors of a hoverboard together.
	No smoothing because it depends on hover STM32 smoothing.
	"""
	MAX_NEG = -10000
	MAX_POS = 10000
	SAFE_REGION = 0.9

	__slots__ = ['pin1', 'pin2', '__rotational', '__translational', 'neutral', 'amplitude', '__steer', '__speed']
	@throws(ValueError, PCAConnectionError)
	def __init__(self, config: tuple, die_on_range_err=False):
		if len(config) != 2:
			raise ValueError("Hover needs config to be a tuple of two pins")
		
		self.pin1 = config[0]
		self.pin2 = config[1]
		self.__rotational = PWM(self.pin1, (Hover.MAX_NEG, Hover.MAX_POS), die_on_range_err)
		self.__translational = PWM(self.pin2, (Hover.MAX_NEG, Hover.MAX_POS), die_on_range_err)

		self.neutral = (Hover.MAX_POS + Hover.MAX_NEG) / 2
		self.amplitude = abs(Hover.MAX_POS - self.neutral) * self.SAFE_REGION

		self.__steer = self.neutral
		self.__speed = self.neutral

	def apply(self, rot: int = None, trans: int = None):
		if rot is None:
			rot = self.__steer

		if trans is None:
			trans = self.__speed

		self.__rotational.apply(rot)
		self.__translational.apply(trans)

		self.__steer = rot
		self.__speed = trans
