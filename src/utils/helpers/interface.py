from abc import ABCMeta

class Interface(ABCMeta):
    def __call__(cls, *args, **kwargs):
        # Create the instance normally
        instance = super().__call__(*args, **kwargs)

        # Get the methods that need to be implemented from the abstract base class
        for name, method in cls.__abstractmethods__.items():
            if name not in cls.__dict__:
                raise TypeError(f"Can't instantiate abstract class {cls.__name__} without {name} method")

            # Check if the signatures match
            interface_method = getattr(cls, name)
            if not InterfaceEnforcementMeta._signatures_match(method, interface_method):
                raise TypeError(f"Method {name} in class {cls.__name__} does not match the interface signature")

        return instance

    @staticmethod
    def _signatures_match(interface_method, implementing_method):
        interface_signature = inspect.signature(interface_method)
        implementing_signature = inspect.signature(implementing_method)
        return interface_signature == implementing_signature