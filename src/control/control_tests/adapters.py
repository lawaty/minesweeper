from testing.interfaces import IHover
from actuators.Hover import Hover

class HoverAdapter(IHover):
    __slots__ = ['__hover']
    def __init__(self, config: tuple, die_on_range_err: bool = False):
        self.__hover = Hover(config, die_on_range_err)

    def apply(self, rot: int = None, trans: int = None):
        self.__hover.apply(rot, trans)