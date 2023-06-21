import numpy as np
from scipy.interpolate import CubicSpline

def findInterval(xPoints, x):
    for idx, xVal in enumerate(xPoints):
        if xVal > x:
            break
    return idx - 1

class SplineInterpolator:
    def __init__(self):
        pass

    def fit(self, xPoints, yPoints):
        self.xPoints = xPoints
        self.spline = CubicSpline(xPoints, yPoints, bc_type="natural")
        self.coeffs = self.spline.c

    def evaluate(self, x):
        interval = findInterval(self.xPoints, x)

        if interval < 0 or interval > self.coeffs.shape[1]:
            print("WARNING: Evaluating spline outside its bounds at value {}".format(x))
            
        if interval < 0:
            interval = 0
        
        ans = 0
        c = self.coeffs[:, interval]
        x -= self.xPoints[interval]

        for idx, cv in enumerate(c[::-1]):
            ans += cv * (x ** idx)
        
        return ans
    
class ArcLengthSpline:
    def __init__(self, numSamples=50, arclengthDt=0.01):
        self.arclengthDt = arclengthDt
        self.numSamples = numSamples

    def fitSpline(self, xPoints, yPoints, plot=False):
        self.xSpline = SplineInterpolator()
        self.ySpline = SplineInterpolator()
        
        self.tPoints = np.arange(0, xPoints.shape[0])
        self.xSpline.fit(self.tPoints, xPoints)
        self.ySpline.fit(self.tPoints, yPoints)

        arclengths, tVals = self.computeTotalArclength()
        totalArclength = np.sum(arclengths)
        self.totalArclength = totalArclength

        self.deltaArclength = totalArclength / self.numSamples

        newTVals = [0]
        currentTheta = self.deltaArclength
        newXPoints = [self.xSpline.evaluate(0)]
        newYPoints = [self.ySpline.evaluate(0)]

        for idx in range(arclengths.shape[0]):
            if arclengths[:idx].sum() >= currentTheta or idx == arclengths.shape[0] - 1:
                currentTheta += self.deltaArclength
                newTVal = tVals[idx - 1]
                newTVals.append(arclengths[:idx].sum())
                newXPoints.append(self.xSpline.evaluate(newTVal))
                newYPoints.append(self.ySpline.evaluate(newTVal))
                
        newXPoints = np.array(newXPoints)
        newYPoints = np.array(newYPoints)

        self.xSpline = SplineInterpolator()
        self.ySpline = SplineInterpolator()
        
        self.xSpline.fit(newTVals, newXPoints)
        self.ySpline.fit(newTVals, newYPoints)
        self.tPoints = np.array(newTVals)
        self.xCoeffs = self.xSpline.coeffs
        self.yCoeffs = self.ySpline.coeffs

        if plot:
            points = []
            for i in range(self.numSamples):
                theta = self.deltaArclength * i
                x, y = self.xSpline.evaluate(theta), self.ySpline.evaluate(theta)
                points.append([x, y])
            points = np.array(points)

            plt.title("Resampled points")
            plt.scatter(points[:, 0], points[:, 1])
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.show()

    def evaluate(self, theta):
        theta = np.clip(theta, 0, self.totalArclength - 0.01)
        
        interval = int(theta // self.deltaArclength)
        if interval >= self.xCoeffs.shape[1]:
            print(self.totalArclength)
            print(self.deltaArclength)
            print(interval)
        ansX, ansY = 0, 0
        cx = self.xCoeffs[:, interval][::-1]
        cy = self.yCoeffs[:, interval][::-1]
        dTheta = theta - interval * self.deltaArclength

        for idx in range(cx.shape[0]):
            polyTerm = dTheta ** idx
            ansX += cx[idx] * polyTerm
            ansY += cy[idx] * polyTerm
        
        return ansX, ansY

    def estimateIntervalArclength(self, interval):
        xCoeffs = self.xSpline.coeffs[:, interval][::-1]
        yCoeffs = self.ySpline.coeffs[:, interval][::-1]

        tVals = np.arange(interval, interval + 1 + self.arclengthDt, self.arclengthDt) - interval

        pointsX = np.zeros(tVals.shape[0])
        pointsY = np.zeros(tVals.shape[0])
        for idx in range(xCoeffs.shape[0]):
            pointsX += xCoeffs[idx] * (tVals ** idx)
            pointsY += yCoeffs[idx] * (tVals ** idx)

        xDiffs = pointsX[:-1] - pointsX[1:]
        yDiffs = pointsY[:-1] - pointsY[1:]
        xDiffs *= xDiffs
        yDiffs *= yDiffs
        distSums = xDiffs + yDiffs
        lengths = np.sqrt(distSums)

        tVals = tVals[:-1] + interval

        return lengths, tVals

    def computeTotalArclength(self):
        arclengths = []
        tVals = []
        for i in range(self.tPoints.shape[0] - 1):
            intervalArclengths, intervalTVals = self.estimateIntervalArclength(i)

            arclengths.extend(intervalArclengths.tolist())
            tVals.extend(intervalTVals.tolist())
        
        arclengths = np.array(arclengths)
        tVals = np.array(tVals)

        return arclengths, tVals
    
    def getPoints(self):
        lengths, _ = self.computeTotalArclength()
        totalLength = np.sum(lengths)
        points = []
        for theta in np.arange(0, totalLength, self.arclengthDt):
            pos = self.evaluate(theta)
            points.append(pos)
        return np.array(points)
