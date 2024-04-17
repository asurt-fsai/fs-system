import numpy as np
import matplotlib.pyplot as plt
import time
from scipy import interpolate
import pandas as pd
import torch

class TrackGenerator:
    
    def __init__(self,):
        pass
    # generate a random circular track
    def generate_track(self,n=100):
        final = 2 * np.pi - np.pi / 6
        theta = np.arange(0, final, final / n)
        r = np.random.uniform(0, 100, n)
        x = [0]
        y = [0]
        for i in range(n):
            x.append(r[i] * np.cos(theta[i]))
            y.append(r[i] * np.sin(theta[i]))
        x = np.array(x)
        y = np.array(y)
        return [x, y]
    # fit spline to track
    def fit_spline(self,x, y, mult=3, per=1):
        n = mult * len(x)
        tck, u = interpolate.splprep([x, y], s=0, per=per)
        u_new = np.linspace(u.min(), u.max(), n)
        x, y = interpolate.splev(u_new, tck)
        return [x, y]
    
    # smooth the track
    def smooth_track(self,x, y, iterations=2):
        for _ in range(iterations):
            x[1:-1] = (x[0:-2] + x[1:-1] + x[2:]) / 3
            y[1:-1] = (y[0:-2] + y[1:-1] + y[2:]) / 3
        return [x, y]

    # calculate the track length
    def _calculate_track_length(self,x, y):
        l = 0
        n = len(x)
        for i in range(-1, n - 1):
            l += np.sqrt((x[i] - x[i + 1]) ** 2 + (y[i] - y[i + 1]) ** 2)
        return l

    # generate cones from track
    def generate_cones(self,x, y, track_width=1, distance_between_cones=1):
        n = len(x)
        l = self._calculate_track_length(x, y)
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
        norm = np.sqrt(nx ** 2 + ny ** 2)
        nx = nx / norm
        ny = ny / norm
        left_track_x = x + track_width * nx
        left_track_y = y + track_width * ny
        left_l = self._calculate_track_length(left_track_x, left_track_y)
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
        right_l = self._calculate_track_length(right_track_x, right_track_y)
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
        print("Number of left cones: ", number_of_left_cones, ", Number of right cones: ", number_of_right_cones)
        return {'blue': [left_x_list, left_y_list], 'yellow': [right_x_list, right_y_list], 'orange': [orange_x_list, orange_y_list]}

    # plot the track
    def plot_track(self,x, y, color='black'):
        plt.plot(x, y, '-', color=color)

    # plot the cones
    def plot_cones(self,x, y, color='blue'):
        
        plt.plot(x, y, 'o', color=color, linestyle='')

    # plot the track and cones
    def plot_(self,track=None, cones=None):
        if all(v is None for v in [track, cones]):
            print("No data to plot")
            return
        plt.figure()
        self.plot_track(track[0], track[1]) if track is not None else None
        if cones is not None:
            for i in cones:
                self.plot_cones(cones[i][0], cones[i][1], color=i)

        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        plt.show()
        