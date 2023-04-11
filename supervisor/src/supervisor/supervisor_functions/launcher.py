#!/usr/bin/python3
"""
.
"""
from typing import List, Optional, Dict
from supervisor.visualizer import Visualizer, IntMarkers  # type: ignore
from .inspector import Inspector
from .module import Module


class Launcher:  # launcherrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
    """
    This class is the main class of the supervisor node. It is responsible for
    - Creating the inspector object and the visualizer object
    - Creating the interactive markers for the modules
    - Updating the data of the modules and visualizing it

    Attributes:
    ----------
    viz : Visualizer
        The visualizer object
    inspector : Inspector
        The inspector object
    intMarkers : IntMarkers
        The interactive markers object
    """

    def __init__(
        self, config: Dict[str, List[Dict[str, str]]], vizTopic: str, btnTopic: str
    ) -> None:
        self.viz = Visualizer(vizTopic)  # /marker/supervisor
        self.config = config
        self.inspector = self.parsing()
        self.btnTopic = btnTopic

    def parsing(self) -> Inspector:
        """
        Parsing the modules.json file and create inspector object
        """
        modulesToPass = []
        for i in self.config["modules"]:
            modulesToPass.append(Module(i["pkg"], i["launch_file"], i["heartbeats_topic"]))
        inspector = Inspector(modulesToPass)

        return inspector

    def update(self, extraText: Optional[List[List[str]]] = None) -> None:
        """
        Updates the data of the modules and visualizes it.
        """
        self.inspector.update()
        data = self.inspector.getData()
        toVizualize = []
        for idx, dataList in enumerate(data):
            toVizualize.append([0, -0.7 * idx, dataList[0], dataList[3]])
            toVizualize.append([2.5, -0.7 * idx, dataList[1], dataList[3]])
            toVizualize.append([5, -0.7 * idx, dataList[2], dataList[3]])
        nElements = len(toVizualize) // 3
        if extraText is not None:
            for idx, txt in enumerate(extraText):
                toVizualize.append([0, -0.7 * (idx + nElements), txt[0], txt[2]])
                toVizualize.append([4, -0.7 * (idx + nElements), txt[1], txt[2]])
        self.viz.visualizeText(toVizualize)

    def launch(self) -> None:
        """
        Launches the modules.
        """
        self.inspector.autoLaunch()
        intMarkers = IntMarkers(self.btnTopic)
        for idx, module in enumerate(self.inspector.modules):
            intMarkers.addButton(7.5, -0.7 * idx, module)
