import Adafruit_GPIO as GPIO
import time
from helpers.Logger import BaseCustomException

class HallEncoder:
    STATES = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]
    REP_PER_REV = 6  # How many times the states repeat during one revolution
    VELOCITY_BUFFER_SIZE = 5  # Avg velocity since the last nth update instead of instantaneous velocity
    CIRCUMFERENCE = 30 # in cm

    __slots__ = ['__gpio', '__halls', '__state', '__revs', '__direction', '__prev_time', '__velocity_buffer']
    def __init__(self, pins: tuple):
        if len(pins) != 3:
            raise ValueError("Hall needs exactly 3 pins to work")
        
        try:
            self.__gpio = GPIO.get_platform_gpio()
        except RuntimeError as e:
            raise GPIOConnectionError(str(e))

        for pin in pins:
            self.__gpio.setup(pin, GPIO.IN)

        self.__halls = pins
        self.__state = self.__get_current_state()
        if self.__state in HallEncoder.STATES:
            self.__state = HallEncoder.STATES.index(self.__state)
        else:
            raise InvalidState(f"Halls state {self.__state} is invalid")

        self.__revs = 0
        self.__direction = None
        self.__prev_time = time.time()
        self.__velocity_buffer = []

    def update(self):
        """
        :throws 
        """
        current_time = time.time()
        state = self.__get_current_state()
        
        if state in HallEncoder.STATES:
            index = HallEncoder.STATES.index(state)
            delta_index = index - self.__state
            if delta_index == 1 or delta_index == -5:  # CW
                self.__direction = 1
            elif delta_index == -1 or delta_index == 5:  # ACW
                self.__direction = -1
            else:
                raise InvalidTransition(f"Hall invalid transition from {self.STATES[self.__state]} to {self.STATES[index]}")
            
            delta_revs = delta_index / len(HallEncoder.STATES) / HallEncoder.REP_PER_REV * self.__direction

            self.__revs += delta_revs
            
            time_diff = current_time - self.__prev_time
            if time_diff > 0:
                velocity = abs(delta_index / time_diff) * self.__direction
                self.__velocity_buffer.append(velocity)
                if len(self.__velocity_buffer) > HallEncoder.VELOCITY_BUFFER_SIZE:
                    self.__velocity_buffer.pop(0)

            self.__state = index
        else:
            raise InvalidState(f"Halls state {state} is invalid")

        self.__prev_time = current_time

    def get_state(self):
        return self.__state
    
    def get_revolutions(self):
        """
        :return moved displacement in revolutions
        """
        return self.__revs
    
    def get_disp(self):
        """
        :return moved displacement in cm
        """
        return self.__revs * self.CIRCUMFERENCE
    
    def get_direction(self):
        return self.__direction
    
    def get_velocity(self):
        if len(self.__velocity_buffer) == 0:
            return 0.0
        return sum(self.__velocity_buffer) / len(self.__velocity_buffer)
    
    def __get_current_state(self):
        return (self.__gpio.input(self.__halls[0]), 
                self.__gpio.input(self.__halls[1]), 
                self.__gpio.input(self.__halls[2]))

class InvalidState(BaseCustomException):
    def __init__(msg):
        super().__init__(msg, "error")

class InvalidTransition(BaseCustomException):
    def __init__(msg):
        super().__init__(msg, "error")

class GPIOConnectionError(BaseCustomException):
    def __init__(msg):
        super().__init__(msg, "error")