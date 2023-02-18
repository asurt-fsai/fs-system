#!/usr/bin/env python3
"""
PID Controller
"""
import rospy

KP = rospy.get_param("/gains/proportional")  # propotional gain
KD = rospy.get_param("/gains/differential")  # differential gain
KI = rospy.get_param("/gains/integral")  # integral gain
DT = rospy.get_param("/time_step")  # [s] time step


def targetSpeedCalc(delta: float) -> float:
    """
    Calculating Target Speed for PID controller
    """
    targetSpeed: float = (20.0 / 3.6) / (abs(delta) * 4)  # [m/s]
    # waypoints.update(pose)
    if targetSpeed <= 15 / 3.6:  # min speed
        targetSpeed = 15 / 3.6
    if targetSpeed >= 60 / 3.6:  # max speed
        targetSpeed = 60 / 3.6
    return targetSpeed


def proportionalControl(
    targetSpeed: float, currentSpeed: float
) -> float:  # longitudinal controller
    """
    PID Controller
    """
    acc: float = (
        KP * (targetSpeed - currentSpeed)
        + KD * ((targetSpeed - currentSpeed) / DT)
        + KI * (targetSpeed - currentSpeed) * DT
    )
    return acc
