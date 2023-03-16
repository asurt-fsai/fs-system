"""
.
"""
import typing
import rospy
import roslaunch
from asurt_msgs.msg import NodeStatus as Status


class Module:
    """
    Holds the required launch file args of a module for inspection.\n
    You can pass a launch file for an accessory (i.e rviz, plotter, etc.)
    Attributes
    ----------
    pkg : str
        The name of the package.
    launch : str
        The name of the launch file.
    heartbeat : str, optional
        Checks if the node is alive.
    """

    def __init__(self, pkg: str, launchFile: str, heartbeat: typing.Any = None) -> None:
        self.pkg = pkg
        self.launchFile = launchFile
        if heartbeat is not None:
            rospy.Subscriber(heartbeat, Status, self.heartbeatCallback)
        self.moduleHandle = None
        self.state = 4  # Shutdown
        self.countSinceLast = 0.0
        self.rate = 0.0
        self.scheduleRestart = 0

    def restart(self) -> None:
        """
        Restarts the module.
        """
        self.shutdown()
        self.launch()

    def launch(self) -> None:
        """
        Launches the module.
        """
        if self.pkg is None or self.launchFile is None:
            return None, None
        self.state = 0  # Launching
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        def launchFile(pkg: str, file: str) -> roslaunch.parent.ROSLaunchParent:
            if None in [pkg, file]:
                return None

            cliArgs = [pkg, file]
            roslaunchFile = roslaunch.rlutil.resolve_launch_arguments(cliArgs)

            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunchFile)
            parent.start()

            return parent

        self.moduleHandle = launchFile(self.pkg, self.launchFile)

    def shutdown(self) -> None:
        """
        Shuts down the module.
        """
        if self.moduleHandle is not None:
            self.moduleHandle.shutdown()
            self.moduleHandle = None
        self.state = 4  # Shutdown

    def heartbeatCallback(self, msg: Status) -> None:
        """
        Checks if the node is alive.

        attributes:
             msg: NodeStatus
        returns:
             None
        """
        self.state = msg.status
        self.countSinceLast += 1.0  # increment the count of messages since last heartbeat

    def update(self) -> None:
        """
        Updates the module.
        """
        if self.scheduleRestart == 1:
            self.restart()
            self.scheduleRestart = 0
        if self.countSinceLast == 0.0 and self.state == 2:
            self.state = 5  # unresponsive
        if self.rate == 0.0:
            self.rate = self.countSinceLast
        else:
            beta = 0.3  # smoothing factor
            self.rate = beta * self.rate + (1.0 - beta) * self.countSinceLast
        self.countSinceLast = 0.0
