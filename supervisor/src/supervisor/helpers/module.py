# pylint: disable=all
# mypy: ignore-errors
"""
Module class to launch and shutdown modules (launch files)
"""
from threading import Thread, Event
from typing import Optional, Callable, Any

import rospy
import roslaunch
from asurt_msgs.msg import NodeStatus


class IntervalTimer(Thread):
    """
    Timer that calls a function at a given interval in a separate thread

    Parameters
    ----------
    interval : float
        The interval to call the function at in seconds
    workerFunc : Callable[..., Any]
        The function to call
    """

    def __init__(self, interval: float, workerFunc: Callable[..., Any]) -> None:
        Thread.__init__(self)
        self.stopEvent = Event()
        self._interval = interval
        self.workerFunc = workerFunc

    def stop(self) -> None:
        """
        Stops the timer and joins the thread
        """
        if self.isAlive():  # pylint: disable = deprecated-method
            # set event to signal thread to terminate
            self.stopEvent.set()
            # block calling thread until thread really has terminated
            self.join()

    def run(self) -> None:
        """
        The infinite loop that calls the function at the given interval
        """
        while not self.stopEvent.is_set():
            self.workerFunc()
            rospy.sleep(self._interval)


class Module:  # pylint: disable = too-many-instance-attributes
    """
    Launches a module (launch file) and checks if it is alive using the optional heartbeat topic
    It can also shutdown and restart the module

    Parameters
    ----------
    pkg : str
        The name of the package.
    launch : str
        The name of the launch file.
    heartbeat : str, optional
        Checks if the node is alive.
    isHeartbeatNodestatus : bool, optional
        If the heartbeat topic is a NodeStatus message, by default True
        If false, the topic is assumed to be the same type as the topic the module publishes
    """

    def __init__(
        self,
        pkg: str,
        launchFile: str,
        heartbeat: Optional[str] = None,
        isHeartbeatNodestatus: bool = True,
    ) -> None:
        self.pkg = pkg
        self.launchFile = launchFile
        self.state = NodeStatus.SHUTDOWN
        self.moduleHandle = None
        self.scheduleRestart = False
        self.hasHeartbeat = heartbeat is not None
        self.rate = 0.0

        if self.hasHeartbeat:
            if isHeartbeatNodestatus:
                rospy.Subscriber(heartbeat, NodeStatus, self.heartbeatCallback)
            else:
                rospy.Subscriber(heartbeat, rospy.AnyMsg, self.msgCallback)

            self.heartbeartRateThread = IntervalTimer(1, self.updateHeartbeatRate)
            self.lastHeartbeatTime = rospy.Time.now().to_sec()
            self.heartbeatCount = 0.0

        # Parameter Validation
        try:
            assert self.pkg is not None and self.launchFile is not None
        except Exception as exc:
            errMsg = "Supervisor.Module: must have a pkg and launch file and not None"
            raise TypeError(errMsg) from exc

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
        self.state = NodeStatus.STARTING
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        roslaunchFile = roslaunch.rlutil.resolve_launch_arguments([self.pkg, self.launchFile])

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunchFile)
        parent.start()

        if self.hasHeartbeat:
            self.heartbeartRateThread.start()

        self.moduleHandle = parent

    def shutdown(self) -> None:
        """
        Shuts down the module.
        """
        if self.moduleHandle is not None:
            self.moduleHandle.shutdown()
            if self.hasHeartbeat:
                self.heartbeartRateThread.stop()
                self.heartbeartRateThread = IntervalTimer(1, self.updateHeartbeatRate)
            self.moduleHandle = None
        self.state = NodeStatus.SHUTDOWN

    def heartbeatCallback(self, msg: NodeStatus) -> None:
        """
        Callback for the heartbeat topic

        Parameters
        ----------
        msg : NodeStatus
            The message from the heartbeat topic
        """
        self.state = msg.status

        # increment the count of messages since last update of the heartbeat rate
        self.heartbeatCount += 1.0

    def msgCallback(self, msg: rospy.AnyMsg) -> None:  # pylint: disable = unused-argument
        """
        Callback for the topic the module publishes (used if no heartbeat topic is available)

        Parameters
        ----------
        msg: Any
            The message from the topic
        """
        self.state = NodeStatus.RUNNING
        # increment the count of messages since last update of the heartbeat rate
        self.heartbeatCount += 1.0

    def updateHeartbeatRate(self) -> None:
        """
        Updates the heartbeat rate, called using a looping thread
        """
        timeDiff = rospy.Time.now().to_sec() - self.lastHeartbeatTime
        self.lastHeartbeatTime = rospy.Time.now().to_sec()
        newHeartbeartRate = self.heartbeatCount / (timeDiff + 0.001)  # avoid division by
        self.heartbeatCount = 0

        beta = 0.3  # smoothing factor
        self.rate = beta * self.rate + (1.0 - beta) * newHeartbeartRate
        if newHeartbeartRate < 1:
            self.state = NodeStatus.UNRESPONSIVE

    def __del__(self) -> None:
        self.shutdown()
