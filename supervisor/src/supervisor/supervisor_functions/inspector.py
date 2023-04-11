#!/usr/bin/python3
"""
.
"""
from typing import List
from .module import Module


class Inspector:
    """
    Inspects a given list of modules (pipeline).
    Attributes
    ----------
    modules : List[Module]
        A pipline of modules.
    """

    def __init__(self, modules: List[Module]) -> None:
        self.modules: List[Module] = modules

    def autoLaunch(self) -> None:
        """
        Automatically, launch the module pipline.
        """
        for module in self.modules:
            module.launch()

    def update(self) -> None:
        """
        Update the state of each module in the pipline.
        """
        for module in self.modules:
            module.update()

    def getData(self) -> List[List[object]]:
        """
        Get the data for each module in the pipline.
        """
        data = []
        states = ["starting", "ready", "running", "error", "shutdown", "unresponsive"]
        for module in self.modules:
            data.append([module.pkg, states[module.state], f"{module.rate:.2f} hz", module.state])
        return data
