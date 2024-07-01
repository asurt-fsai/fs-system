"""
Simulation to test FastSLAM algorithm
"""

import numpy as np
import matplotlib.pyplot as plt
from fastslam import FastSLAM
from fastslam import Particle as Particle
from icecream import ic
from utils.utils import angleToPi
import time as tm
landmarks = np.array([[6, 4, 0], [2, 2, 0], [8, 2, 1], [2, 8, 1], [8, 8, 2]])

# Create FastSLAM object
Q = np.diag([0.1, 0.1])**2         # measurement variance
R = np.diag([1.0, 1.0, np.deg2rad(20.0)])**2    # action variance

STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # landmark srate size [x,y]
N_PARTICLE = 10 # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

fs = FastSLAM(N_PARTICLE)

ground_truth = np.array([0, 0, 0])


#plot
def plot_fastslam(ground_truth,landmarkEst,xs, maxP, sensor, pause_time):
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    
    #plot landmark estimate
    for i in range(len(landmarkEst)):
        plt.plot(landmarkEst[i, 0], landmarkEst[i, 1], "ok")

    #plot ground truth
    plt.plot(ground_truth[0],ground_truth[1], ".b")
    plt.quiver(ground_truth[0], ground_truth[1], np.cos(ground_truth[2]), np.sin(ground_truth[2]), color='b', alpha=1.0)
    
    # plot particles
    for i in range(len(xs)):
        # particles
        plt.plot(xs[i].x, xs[i].y, ".r")
        plt.quiver(xs[i].x, xs[i].y, np.cos(xs[i].yaw), np.sin(xs[i].yaw), color='r', alpha=xs[i].weight)
        # landmarks
        for i in range(len(xs)):
            for j in range(xs[i].maxlmID):
                # if landmarkColors[j] == 0 --> blue
                # if landmarkColors[j] == 1 --> yellow
                # if landmarkColors[j] == 2 --> orange
                if xs[i].landmarkColors[j] == 0:
                    plt.plot(xs[i].landmark[j, 0], xs[i].landmark[j, 1], "xb")
                elif xs[i].landmarkColors[j] == 1:
                    plt.plot(xs[i].landmark[j, 0], xs[i].landmark[j, 1], "xy")
                else:
                    plt.plot(xs[i].landmark[j, 0], xs[i].landmark[j, 1], "xr")

    plt.plot(maxP.x, maxP.y, ".r")
    plt.quiver(maxP.x, maxP.y, np.cos(maxP.yaw), np.sin(maxP.yaw), color='g', alpha=1)
        # landmarks
    for j in range(maxP.maxlmID):
        plt.plot(maxP.landmark[j, 0], maxP.landmark[j, 1], "*g")

    # plot landmark
    for i in range(len(sensor)):
        plt.plot(sensor[i, 0], sensor[i, 1], "xg")

    plt.axis("equal")
    plt.grid(True)
    plt.pause(pause_time)

def observation(xs, landmarks):
    zlist = []
    for i in range(landmarks.shape[0]):
        observation = [landmarks[i, 0] - xs[0], landmarks[i, 1] - xs[1], landmarks[i,2]]
        zlist.append(observation)
    zlist = np.array(zlist)
    return zlist

def motionModel(x,controlAction, dT):
    '''
    Simple motion model for the particle

    Parameters
    ----------
    particle: Particle
        The particle to be updated

    controlAction: np.ndarray, shape=(2,1)
        Control action [v, omega]

    Returns
    -------
    particle: Particle
        Updated particle
    '''
    uTheta = controlAction[1] * dT
    uX = controlAction[0] * dT * np.cos(x[2])
    uY = controlAction[0] * dT * np.sin(x[2])
    return np.array([x[0] + uX, x[1] + uY, x[2] + uTheta])

def estimateLandmark(x, obs):
        observation = obs[0] + 1j * obs[1]
        dist = abs(observation)
        angle = np.angle(observation)
        observation = [dist, angle, obs[2]]

        r = observation[0]
        b = observation[1] - x[2]
    
        s = np.sin(angleToPi(x[2] + b))
        c = np.cos(angleToPi(x[2] + b))

        return [x[0] + r * c, x[1] + r * s]

if __name__ == '__main__':
    time = 0.0
    while time <= 50.0:
        time += 100e-3

        # input
        u = np.array([1.0, np.deg2rad(5.0)])  # [v, yawrate]
# ground truth
        ground_truth = motionModel(ground_truth, u,100e-3)
        
        # observation
        zlist = observation(ground_truth[:2], landmarks)

        landmarkEstimate = np.zeros((len(zlist), 2))
        # estimate landmark position
        for i in range(len(zlist)):
            landmarkEstimate[i] = estimateLandmark(ground_truth, zlist[i])
            
        zlist[:,:2] += np.random.randn(*zlist[:,:2].shape) @ np.linalg.cholesky(Q).T
        # FastSLAM
        fs.update(u, zlist, 100e-3)
        # plot
        plot_fastslam(ground_truth,landmarkEstimate,fs.particles, fs.maxWeightParticle, landmarks, 0.1)
        