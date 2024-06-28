#!/usr/bin/env python3

from abc import abstractmethod
from helpers.Logger import throws
from helpers.interface import Interface
from testing.exceptions import PCAConnectionError, OutOfRangeSignal

class IHoverManual(metaclass=Interface):
    @abstractmethod
    @throws(ValueError, PCAConnectionError)
    def __init__(self, config: tuple, die_on_range_error: bool = False):
        """
        config must be exactly a tuple of two integers(pin no.) for steer and speed.

        @throws ValueError for invalid config structure
        @throws PCAConnectionError if pwm driver isn't initialized correctly
        """
        pass

    @abstractmethod
    @throws(OutOfRangeSignal)
    def apply(self, rot: int = None, trans: int = None):
        """
        Method applies rotational pwm signal and translational pwm signal optionally. If rot or trans is not given, just leave it as it is
        """
        pass
