from testing.interfaces import IHoverAutomated, IHoverManual
from actuators.Hover import Hover


class HoverManualAdapter(IHoverManual):
    __slots__ = ['__hover']
    def __init__(self, config: tuple, die_on_range_err: bool = False):
        self.__hover = Hover(config, die_on_range_err)

    def apply(self, rot: int = None, trans: int = None):
        self.__hover.apply(rot, trans)

class HoverAutomatedAdapter(IHoverAutomated):
    __slots__ = ['__hover']
    def __init__(self, config: tuple, die_on_range_err: bool = False):
        self.__hover = Hover(config, die_on_range_err)

    def apply(self, rot: int = None, trans: int = None):
        self.__hover.apply(rot, trans)
