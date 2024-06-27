import rospy
import os
import logging
from datetime import datetime
from utils.msg import Log  # assuming Log message structure is defined in utils.msg
from traceback import format_exc as trace
from functools import wraps
from helpers.Logger import *

class BaseCustomException(Exception):
    def __init__(self, msg, severity):
        self.severity = severity
        super().__init__(msg)

def throws(*exceptions):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        exception_list = ', '.join([exc.__name__ for exc in exceptions])
        wrapper.__doc__ = f"{func.__doc__}\n\nThrows: {exception_list}" if func.__doc__ else f"Throws: {exception_list}"
        return wrapper
    return decorator

class Logger:
    __inst = None

    def __init__(self):
        rospy.init_node("logs", anonymous=True)
        self.__pub = rospy.Publisher("logs", Log, queue_size=10)

        # Logging Persistance
        logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(logs_dir, exist_ok=True)

        log_file = datetime.now().strftime("%Y-%m-%d.log")
        log_path = os.path.join(logs_dir, log_file)
        logging.basicConfig(filename=log_path, level=logging.DEBUG,
                            format='%(asctime)s - %(levelname)s - %(message)s')

    @staticmethod
    def getInst() -> 'Logger':
        if not Logger.__inst:
            Logger.__inst = Logger()
        return Logger.__inst

    def log(self, msg: str, severity: str = "info"):
        log = Log()
        log.severity = severity
        log.msg = msg
        log.trace = trace()
        
        self.__pub.publish(log)
        logging.info(f"[{severity.upper()}] {msg}")

    def logE(self, e: BaseCustomException):
        log = Log()
        log.severity = e.severity
        log.msg = f"{e.__class__.__name__}: " + str(e)
        log.trace = trace()

        self.__pub.publish(log)
        logging.info(f"[{e.severity.upper()}] {e.__class__.__name__}: {str(e)}")