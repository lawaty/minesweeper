from abc import ABCMeta, abstractmethod
import inspect


class Interface(ABCMeta):
    def __call__(cls, *args, **kwargs):
        instance = super().__call__(*args, **kwargs)

        method_list = [
            func
            for func in dir(cls)
            if callable(getattr(cls, func)) and not func.startswith("__")
        ]
        method_list.append("__init__")

        for name in method_list:
            if not hasattr(cls, name):
                raise TypeError(
                    f"Can't instantiate abstract class {cls.__name__} without {name} method"
                )

            interface_method = getattr(cls, name)
            abstract_method = getattr(cls.__bases__[0], name)
            if not Interface._signatures_match(abstract_method, interface_method):
                raise TypeError(
                    f"Method {name} in class {cls.__name__} does not match the interface signature"
                )

        return instance

    @staticmethod
    def _signatures_match(interface_method, implementing_method):
        interface_signature = inspect.signature(interface_method)
        implementing_signature = inspect.signature(implementing_method)
        return interface_signature == implementing_signature
