import numpy as np
# from fastslam import HungarianAlg

# import types for numpy arrays
from typing import List
from math import cos, sin
import math
from utils.utils import angleToPi

Q = np.diag([3.0, np.deg2rad(10.0)])**2         # measurement variance
R = np.diag([1.0, 1.0, np.deg2rad(20.0)])**2    # action variance

STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # landmark srate size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

class Particle:

    def __init__(self, N_LM):
        self.weight = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # landmark x-y positions
        self.landmark = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))
        self.maxlmID = 0

class FastSLAM:
    def __init__(self, numParticles, initPose=np.zeros((1, 3))):
        self.pose = initPose.reshape(-1, 3) # x, y, theta
        self.dT = 100e-3
        self.R = np.diag([0.1, np.deg2rad(20.0)])**2
        self.Q = np.diag([3.0, np.deg2rad(10.0)])**2
        self.numParticles = numParticles


    ##########    
    # TESTED #
    ##########

    def predictParticles(self,particles,u):
        '''
        Predict the particles using the motion model
        @param particles: list of particles
        @param u: control input [v, omega]
        @return: list of particles
        '''
        for i in range(particles.shape[0]):
            noisyAction = u + (np.random.randn(1, 2) @ self.R).T # [v, omega]
            particles[i] = self.motionModel(particles[i], noisyAction)
        return particles
   
    def motionModel(self,particle,u):
        uTheta = u[1, 0] * self.dT
        particle.yaw += uTheta
        uX = u[0, 0] * self.dT * cos(particle.yaw)
        uY = u[1, 0] * self.dT * sin(particle.yaw)
        particle.x += uX
        particle.y += uY
        return particle
   
    def calcJacobian(self,particle, xf, Pf, Q):
        """
        Calculates the jacobian 

        Parameters
        ----------
        particle: Particle
            The particle of which the jacobian will be calculated

        xf: np.ndarray, shape=(2,1)
            position of the landmark

        Pf: np.ndarray, shape=(2,2)
            Covariance of the landmark's position

        Q: np.ndarray, shape=(2,2)
            Covariance of the measurements
            
        Returns
        -------
        estimateObservation: np.ndarray, shape=(2,1)
            Estimated observation of the landmark from particle [distance, bearing].

        Hv: np.ndarray, shape=(2,3)
            Jacobian of the matrix

        Hf: np.ndarray, shape=(2,2)
            Jacobian ???

        Sf: np.ndarray, shape
            Covariance
        """

        dx = xf[0, 0] - particle.x
        dy = xf[1, 0] - particle.y
        d2 = dx**2 + dy**2
        d = math.sqrt(d2)

        estimateObservation = np.array(
            [d, angleToPi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

        Hv = np.array([[-dx / d, -dy / d, 0.0],
                    [dy / d2, -dx / d2, -1.0]])

        Hf = np.array([[dx / d, dy / d],
                    [-dy / d2, dx / d2]])

        Sf = Hf @ Pf @ Hf.T + Q

        return estimateObservation, Hv, Hf, Sf
    

    def calcWeightSingle(self,particle: Particle, observation, Q):
        '''
        Calculates the weight for a particle given a single landmark observation,
        particle weight is sum of weights for each landmark

        Parameters
        ----------
        particle: Particle
            The particle of which the jacobian will be calculated

        observation: np.ndarray, shape=(2,1)
            Observation of landmark [distance,bearing]

        Q: np.ndarray, shape=(2,2)
            Covariance of the measurements
            
        Returns
        -------
        w: float
            Sample weight of the particle given a single landmark
        '''

        landmarkID = int(observation[2])
        landmarkState = np.array(particle.landmark[landmarkID, :]).reshape(2, 1)
        landmarkCovar = np.array(particle.lmP[2 * landmarkID:2 * landmarkID + 2])
        estimateObservation, _, _, covariance = self.calcJacobian(particle, landmarkState, landmarkCovar, Q)
        dObservation = observation[0:2].reshape(2, 1) - estimateObservation
        dObservation[1, 0] = angleToPi(dObservation[1, 0])

        try:
            invCovar = np.linalg.inv(covariance)
        except np.linalg.linalg.LinAlgError:
            print("singuler")
            return 1.0

        num = math.exp(-0.5 * dObservation.T @ invCovar @ dObservation)
        den = math.sqrt(np.linalg.det(2.0 * math.pi * covariance))
        w = num / den

        return w


    def normalizeWeight(self,particles):
        """
        Calculates the jacobian 

        Parameters
        ----------
        particles: List(Particle)
            List of particles with non-normalized weights
            
        Returns
        -------
        particles: List(Particle)
            Updated particles with normalized weights
        
        """

        sumWeight = sum([p.weight for p in particles])

        try:
            for i in range(self.numParticles):
                particles[i].weight /= sumWeight
        except ZeroDivisionError:
            for i in range(self.numParticles):
                particles[i].weight = 1.0 / self.numParticles
            return particles
        return particles


    def updateKF(self,xf, Pf, v, Q, Hf):
        PHt = Pf @ Hf.T
        S = Hf @ PHt + Q

        S = (S + S.T) * 0.5
        SChol = np.linalg.cholesky(S).T
        SCholInv = np.linalg.inv(SChol)
        W1 = PHt @ SCholInv
        W = W1 @ SCholInv.T

        x = xf + W @ v
        P = Pf - W1 @ W1.T

        return x, P


    def updateLandmark(self,particle: Particle, observation, Q):

        landmarkID = int(observation[2])
        xf = np.array(particle.landmark[landmarkID, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * landmarkID:2 * landmarkID + 2, :])

        estimateObservation, _, Hf, _ = self.calcJacobian(particle, xf, Pf, Q)

        dz = observation[0:2].reshape(2, 1) - estimateObservation
        dz[1, 0] = angleToPi(dz[1, 0])

        xf, Pf = self.updateKF(xf, Pf, dz, Q, Hf)

        particle.landmark[landmarkID, :] = xf.T
        particle.lmP[2 * landmarkID:2 * landmarkID + 2, :] = Pf

        return particle
        

    def updateWithObservation(self,particles, observation):
        for iz in range(len(observation[0, :])):

            lmid = int(observation[2, iz])

            for indParticle in range(self.numParticles):
                # new landmark
                if abs(particles[indParticle].landmark[lmid, 0]) <= 0.01:
                    particles[indParticle] = self.addNewLandmark(particles[indParticle], observation[:, iz], self.Q)
                # known landmark
                else:
                    w = self.calcWeightSingle(particles[indParticle], observation[:, iz], self.Q)
                    particles[indParticle].weight *= w
                    particles[indParticle] = self.update_landmark(particles[indParticle], observation[:, iz], self.Q)

        return particles
        


    def addNewLandmark(self,particle, observation, Q):

        r = observation[0]
        b = observation[1]
        landmarkID = int(observation[2])

        s = math.sin(angleToPi(particle.yaw + b))
        c = math.cos(angleToPi(particle.yaw + b))

        particle.landmark[landmarkID, 0] = particle.x + r * c
        particle.landmark[landmarkID, 1] = particle.y + r * s

        # covariance
        Gz = np.array([[c, -r * s],
                    [s, r * c]])

        particle.lmP[2 * landmarkID:2 * landmarkID + 2] = Gz @ Q @ Gz.T

        return particle
    
    def resampling(self,particles):
        """
        low variance re-sampling
        """

        particles = self.normalizeWeight(particles)
        maxWeightIndex = np.argmax([p.weight for p in particles])
        particleWeights = []
        for i in range(self.numParticles):
            particleWeights.append(particles[i].weight)

        particleWeights = np.array(particleWeights)

        effParticleNumber = 1.0 / (particleWeights @ particleWeights.T)  # Effective particle number
        # print(effParticleNumber)

        if effParticleNumber < NTH:  # resampling
            cumWeight = np.cumsum(particleWeights)
            base = np.cumsum(particleWeights * 0.0 + 1 / self.numParticles) - 1 / self.numParticles
            resampleid = base + np.random.rand(base.shape[0]) / self.numParticles

            inds = []
            ind = 0
            for indParticle in range(self.numParticles):
                while ((ind < cumWeight.shape[0] - 1) and (resampleid[indParticle] > cumWeight[ind])):
                    ind += 1
                inds.append(ind)

            tparticles = particles[:]
            for i in range(len(inds)):
                particles[i].x = tparticles[inds[i]].x
                particles[i].y = tparticles[inds[i]].y
                particles[i].yaw = tparticles[inds[i]].yaw
                particles[i].weight = 1.0 / self.numParticles

        return particles, inds, maxWeightIndex
    

    ##############    
    # NOT TESTED #
    ##############