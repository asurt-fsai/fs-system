#!/usr/bin/env python3

"""
PID controller module used to control the throttle and velocity of the vehicle


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
KP = 0.5  # rospy.get_param("/gains/proportional")  # propotional gain
KD = 0.4  # rospy.get_param("/gains/differential")  # differential gain
KI = 0.5  # rospy.get_param("/gains/integral")  # integral gain
DT = 0.1  # rospy.get_param("/time_step")  # [s] time step
MAXSPEED = 20  # rospy.get_param("/max_speed")  # [m/s] max speed
MINSPEED = 2  # rospy.get_param("/min_speed")  # [m/s] min speed
TARGETSPEED = 10  # rospy.get_param("/target_speed")  # [m/s] target speed
MAXACC = 15  # rospy.get_param("/max_acceleration")  # [m/ss] max acceleration
MINACC = -15  # rospy.get_param("/min_acceleration")  # [m/ss] min acceleration


def targetSpeedCalc(delta: float) -> float:
    """
    calculates the target speed based on the curvature of the path

    Parameters
    ----------
    delta : float
        steering angle of the vehicle

    Returns
    -------
    targetSpeed : float
        calculated target speed of the vehicle
    """
    targetSpeed: float = TARGETSPEED / (abs(delta) * 4 + 0.0001)  # [m/s]
    # waypoints.update(pose)
    targetSpeed = max(targetSpeed, MINSPEED)
    targetSpeed = min(targetSpeed, MAXSPEED)
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
    # if acc >= MAXACC:
    #     acc = MAXACC
    # if acc <= MINACC:
    #     acc = MINACC
    return acc


def throttleControl(acc: float) -> float:
    """
    Throttle Control
    """

    throttle: float = acc / MAXACC
    return throttle
