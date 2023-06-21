import os
import yaml
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

from .spline_interpolator import ArcLengthSpline

class Waypoint:
    def __init__(self, pos):
        self.positions = [pos]
    
    def addReading(self, pos):
        self.positions.append(pos)
    
    def get(self):
        return np.mean(self.positions, axis=0)
        
class WaypointsCleaner:
    def __init__(self, toRear, paramPath=None):
        self.toRear = np.array(toRear)

        if paramPath is None:
            dirname = os.path.dirname(__file__)
            paramPath = os.path.join(dirname, 'params.yaml')
            
        with open(paramPath, "r") as f:
            params = yaml.safe_load(f)

        for key in params.keys():
            setattr(self, key, params[key])
        self.reset()
        
    def reset(self):
        self.currPos = np.zeros(2)
        self.smoothPrev = np.zeros((1, 2))
        self.currHeading = 0
        self.prevWaypoints = []
        self.prevDoneWaypoints = []
        self.passedWaypoints = []

    def updatePosition(self, currPos, currHeading):
        newPos = np.array(currPos)
        if np.linalg.norm(newPos - self.currPos) > 2:
            self.reset()
        self.currPos = newPos
        self.currHeading = currHeading
        for idx, waypoint in enumerate(self.prevWaypoints):
            position = waypoint.get()
            if np.linalg.norm(position - self.currPos) < 1:
                self.prevDoneWaypoints.append(idx)
        
    def addWaypoints(self, waypoints):
        waypoints = np.array(waypoints)
        s, c = np.sin(self.currHeading), np.cos(self.currHeading)
        rotMat = np.array([[c, -s], [s, c]])
        waypoints = (rotMat @ waypoints.transpose()).transpose()
        waypoints = np.array(waypoints) + self.currPos.reshape(1, 2)

        positions = []
        for waypoint in self.prevWaypoints:
            positions.append(waypoint.get())
        positions = np.array(positions)
        
        for waypoint in waypoints:
            if positions.shape[0] > 0:
                dists = np.linalg.norm(positions - waypoint, axis=1)
                minDist, minDistIdx = np.min(dists), np.argmin(dists)
                if minDist < self.minDistUnique:
                    self.prevWaypoints[minDistIdx].addReading(waypoint)
                else:
                    self.prevWaypoints.append(Waypoint(waypoint))
            else:
                self.prevWaypoints.append(Waypoint(waypoint))
                
        
    def getOrderedWaypoints(self, waypoints):
        s, c = np.sin(self.currHeading), np.cos(self.currHeading)
        rotMat = np.array([[c, -s], [s, c]])
        vecRear = (rotMat @ self.toRear.transpose()).transpose()
        rearPos = self.currPos + vecRear
        orderedWaypoints = [rearPos, self.currPos]
        currWaypoint = self.currPos
        while waypoints.shape[0] > 0:
            dists = np.linalg.norm(waypoints - currWaypoint, axis=1)
            minDist, minDistIdx = np.min(dists), np.argmin(dists)
            if minDist > self.euclideanDistOutlier:
                break
                
            currWaypoint = np.copy(waypoints[minDistIdx])
            orderedWaypoints.append(waypoints[minDistIdx].tolist())
            
            waypoints = np.delete(waypoints, minDistIdx, axis=0)
        return np.array(orderedWaypoints)
            
    
    def fitSpline(self, waypoints):
        k = 2
        if waypoints.shape[0] < 4:
            k = waypoints.shape[0] - 1
            
        if k < 1:
            return waypoints
            
        tck, u = interpolate.splprep(waypoints.T, w=np.ones(waypoints.shape[0]), s=self.interpolateSmoothing, k=k)
        unew = np.linspace(0, 1, self.numInterpolationPoints)
        newWaypoints = np.array(interpolate.splev(unew, tck)).T
        return newWaypoints
        
    def getWaypoints(self):
        positions = []
        numReadings = []
        
        carForwardVector = np.array([np.cos(self.currHeading), np.sin(self.currHeading)])
        
        for idx, waypoint in enumerate(self.prevWaypoints):
            waypointPos = waypoint.get()
            if idx not in self.prevDoneWaypoints and len(waypoint.positions) > self.minObservationsBeforeUsingWaypoint:
                isFront = np.dot(carForwardVector, waypointPos - self.currPos)
                
                if isFront > 0 or not self.filterBehindCar:
                    positions.append(waypointPos)
                    numReadings.append(np.clip(len(waypoint.positions), 0, self.maxNumReadings))
            
        if len(positions) == 0:
            return np.array([])
        meanNumReadings = np.mean(numReadings)
        
        waypointsToUse = []
        for pos, read in zip(positions, numReadings):
            if read >= meanNumReadings - self.meanNumReadingsSlack:
                waypointsToUse.append(pos)
        waypointsToUse = np.array(waypointsToUse)
        
        if waypointsToUse.shape[0] == 0:
            return np.array([])
            
        orderedWaypoints = self.getOrderedWaypoints(waypointsToUse)

        newWaypoints = self.fitSpline(orderedWaypoints)
        if newWaypoints.shape[0] < 4:
            return newWaypoints
        
        acSpline = ArcLengthSpline(numSamples=10, arclengthDt=0.1)
        acSpline.fitSpline(newWaypoints[:, 0], newWaypoints[:, 1])
        allWaypoints = acSpline.getPoints()
        return allWaypoints

        x, y = acSpline.evaluate(self.thetaLookahead)
        nextWaypoint = np.array([[x, y]])
        smoothed = nextWaypoint * (1 - self.expParam) + self.expParam * self.smoothPrev
        self.smoothPrev = smoothed
        
        if self.plot:
            plt.clf()
            plt.xlim([-50, 50])
            plt.ylim([-100, 100])
            plt.plot(newWaypoints[:, 0], newWaypoints[:, 1], c='c')
            plt.scatter(waypointsToUse[:, 0], waypointsToUse[:, 1], c='g', marker='x')
            plt.scatter(smoothed[:, 0], smoothed[:, 1], c='k', marker='^')
            plt.pause(0.05)
        
        return [[*self.currPos], smoothed[0]]
