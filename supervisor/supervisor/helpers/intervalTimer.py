"""
Interval timer to call a function at a given interval in a separate thread
"""

from threading import Thread, Event
from typing import Callable, Any

import rclpy


class IntervalTimer(Thread):
    """
    Timer that calls a function at a given interval in a separate thread

    Parameters
    ----------
    interval : float
        The interval to call the function at in seconds
    worker_func : Callable[..., Any]
        The function to call
    """

    def __init__(self, interval: float, worker_func: Callable[..., Any]) -> None:
        Thread.__init__(self)
        self.stopEvent = Event()
        self._interval = interval
        self._workerFunc = worker_func

    def stop(self) -> None:
        """
        Stops the timer and joins the thread
        """
        if self.is_alive():
            # set event to signal thread to terminate
            self.stopEvent.set()
            # block calling thread until thread really has terminated
            self.join()

    def run(self) -> None:
        """
        The infinite loop that calls the function at the given interval
        """
        while not self.stopEvent.is_set():
            self._workerFunc()
            rclpy.sleep(self._interval)
