#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
import math

# from tqdm import tqdm
from scipy import interpolate
from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

s = Path


# generate a random circular track
def generate_track(n=100):
    final = 2 * np.pi - np.pi / 6
    # generate random angles
    theta = np.arange(0, final, final / n)
    # generate random radii
    r = np.random.uniform(0, 100, n)
    # print(r)
    x = [0]
    y = [0]
    for i in range(n):
        x.append(r[i] * np.cos(theta[i]))
        y.append(r[i] * np.sin(theta[i]))
    x = np.array(x)
    y = np.array(y)
    return [x, y]


# fit spline to track
def fit_spline(x, y, mult=3, per=1):
    # print(x,y)
    n = mult * len(x)
    # fit spline
    tck, u = interpolate.splprep([x, y], s=0, per=per)
    u_new = np.linspace(u.min(), u.max(), n)
    x, y = interpolate.splev(u_new, tck)

    return [x, y]


# smooth the track
def smooth_track(x, y, iter=2):
    # smooth the track
    for i in range(iter):
        x[1:-1] = (x[0:-2] + x[1:-1] + x[2:]) / 3
        y[1:-1] = (y[0:-2] + y[1:-1] + y[2:]) / 3

    return [x, y]


# calculate the track length
def calculate_track_length(x, y):
    # track length
    l = 0
    n = len(x)
    # print(n)
    for i in range(-1, n - 1):
        try:
            l += np.sqrt((x[i] - x[i + 1]) ** 2 + (y[i] - y[i + 1]) ** 2)
        except:
            print(i, l)
    return l


# generate cones from track
def generate_cones(x, y, track_width=1, distance_between_cones=1):
    n = len(x)
    # track length
    l = calculate_track_length(x, y)
    print("Track Length: ", l)  # ,", Number of cones: ",n_cones)
    # track normals
    nx = np.zeros(n)
    ny = np.zeros(n)
    for i in range(n):
        if i == 0:
            nx[i] = -(y[i + 1] - y[i])
            ny[i] = x[i + 1] - x[i]
        elif i == n - 1:
            nx[i] = -(y[i] - y[i - 1])
            ny[i] = x[i] - x[i - 1]
        else:
            nx[i] = -(y[i + 1] - y[i - 1])
            ny[i] = x[i + 1] - x[i - 1]
    # normalize
    norm = np.sqrt(nx**2 + ny**2)
    nx = nx / norm
    ny = ny / norm
    left_track_x = x + track_width * nx
    left_track_y = y + track_width * ny
    left_l = calculate_track_length(left_track_x, left_track_y)
    number_of_left_cones = int(left_l / distance_between_cones)
    x0, y0 = left_track_x[0], left_track_y[0]
    orange_x_list = [x0]
    orange_y_list = [y0]
    left_x_list = [x0]
    left_y_list = [y0]
    left_nx_list = [nx[0]]
    left_ny_list = [ny[0]]
    for i in range(n):
        dist = np.sqrt((left_track_x[i] - x0) ** 2 + (left_track_y[i] - y0) ** 2)
        if dist >= distance_between_cones:
            left_x_list.append(left_track_x[i])
            left_y_list.append(left_track_y[i])
            left_nx_list.append(nx[i])
            left_ny_list.append(ny[i])
            x0, y0 = left_track_x[i], left_track_y[i]

    right_track_x = x - track_width * nx
    right_track_y = y - track_width * ny
    right_l = calculate_track_length(right_track_x, right_track_y)
    number_of_right_cones = int(right_l / distance_between_cones)
    x0, y0 = right_track_x[0], right_track_y[0]
    orange_x_list.append(x0)
    orange_y_list.append(y0)
    right_x_list = [x0]
    right_y_list = [y0]
    right_nx_list = [nx[0]]
    right_ny_list = [ny[0]]
    for i in range(n):
        dist = np.sqrt((right_track_x[i] - x0) ** 2 + (right_track_y[i] - y0) ** 2)
        if dist >= distance_between_cones:
            right_x_list.append(right_track_x[i])
            right_y_list.append(right_track_y[i])
            right_nx_list.append(nx[i])
            right_ny_list.append(ny[i])
            x0, y0 = right_track_x[i], right_track_y[i]
    print(
        "Number of left cones: ",
        number_of_left_cones,
        ", Number of right cones: ",
        number_of_right_cones,
    )
    return {
        "blue": [left_x_list, left_y_list],
        "yellow": [right_x_list, right_y_list],
        "orange": [orange_x_list, orange_y_list],
    }


# plot the track
def plot_track(x, y, color="black"):
    plt.plot(x, y, "-", color=color)


# plot the cones
def plot_cones(x, y, color="blue"):
    plt.plot(x, y, "o", color=color)


# plot the track and cones
def plot_(track=None, cones=None):
    if all(v is None for v in [track, cones]):
        print("No data to plot")
        return
    plt.figure()
    plot_track(track[0], track[1]) if track is not None else None
    if cones is not None:
        for i in cones:
            plot_cones(cones[i][0], cones[i][1], color=i)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.show()


track = generate_track(20)
smoothed = smooth_track(*track, 4)
spline = fit_spline(*smoothed, 20)
# plot_(track=spline)
# print(len(spline[0]))
x_coord = spline[0]
y_coord = spline[1]
# x_coord = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
# y_coord = [0, 0, 0, 0, 0]
# spline [array of x, array of y]
rospy.init_node("waypointsgen", anonymous=True)
rate = rospy.Rate(40)  # hz
path = Pose()
path_pub = rospy.Publisher("/waypoints", Pose, queue_size=10)
print("started generating waypoints")
if __name__ == "__main__":

    tick = time.time()
    # rate.sleep()
    for i in range(len(x_coord)):
        if i % 1 == 0:
            # y_coord.append(20 * math.sin(x_coord[i]/3))
            path.position.x = x_coord[i]
            path.position.y = y_coord[i]

            path_pub.publish(path)
            rate.sleep()
    tock = time.time()
    print("Time taken: ", tock - tick)
    print("The track is complete")
