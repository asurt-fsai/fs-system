#!/usr/bin/env python3

"""
PID controller module used to as the longitudinal control of the vehicle and gives output
the throttle and velocity of the vehicle


parameters
----------
KP : float
    proportional gain for the PID controller
KD : float
    differential gain for the PID controller
KI : float
    integral gain for the PID controller
DT : float
    time step for the PID controller
MAXSPEED : float
    maximum speed allowed to be set for the vehicle
MINSPEED : float
    minimum speed allowed to be set for the vehicle
TARGETSPEED : float
    constant target speed for the vehicle
MAXACC : float
    maximum acceleration allowed to be set for the vehicle
MINACC : float
    minimum acceleration allowed to be set for the vehicle
"""
import math
from typing import Tuple
import rospy

KP = rospy.get_param("/gains/proportional")  # propotional gain
KD = rospy.get_param("/gains/differential")  # differential gain
KI = rospy.get_param("/gains/integral")  # integral gain
DT = rospy.get_param("/time_step")  # [s] time step
MAXSPEED = rospy.get_param("/speed/max")  # [m/s] max speed
MINSPEED = rospy.get_param("/speed/min")  # [m/s] min speed
TARGETSPEED = rospy.get_param("/speed/target")  # [m/s] target speed
MAXACC = rospy.get_param("/acceleration/max")  # [m/ss] max acceleration
MINACC = rospy.get_param("/acceleration/min")  # [m/ss] min acceleration


class PidController:
    """
    PID Controller class for longitudinal control of the vehicle
    """

    def __init__(self, error: float = 0.0, acc: float = 0.0, throttle: float = 0.0) -> None:
        """
        parameters
        ----------
        error : float
            error between the target speed and the current speed

        acc : float
            acceleration to be applied to the vehicle

        throttle : float
            throttle to be applied to the vehicle
        """
        self.error: float = error
        self.acc: float = acc
        self.throttle: float = throttle

    def proportionalControl(
        self, targetSpeed: float, currentSpeed: float, prevError: float
    ) -> Tuple[float, float]:  # longitudinal controller
        """
        PID Controller for longitudinal control of the vehicle

        Parameters
        ----------
        targetSpeed : float
            target speed for the vehicle

        currentSpeed : float
            current speed of the vehicle

        prevError : float
            previous error between the target speed and the current speed

        Returns
        -------
        acc : float
            acceleration to be applied to the vehicle
        """
        self.error = targetSpeed - currentSpeed

        self.acc = (
            KP * self.error
            + KD * ((self.error - prevError) / DT)
            + KI * (prevError + (self.error * DT))
        )

        return self.acc, self.error

    def throttleControl(self, acc: float) -> float:
        """
        Throttle Control to convert acceleration to throttle,
        to regulate acceleration to be within the limits [-1,1]

        Parameters
        ----------
        acc : float
            acceleration result from the PID controller

        Returns
        -------
        throttle : float
            throttle to be applied to the vehicle
        """

        self.throttle = float(min(math.tanh(acc), math.tanh(MAXACC)))
        return self.throttle
