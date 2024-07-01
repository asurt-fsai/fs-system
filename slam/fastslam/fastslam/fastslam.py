import numpy as np
from icecream import ic
# from fastslam import HungarianAlg

# import types for numpy arrays
from typing import List
from math import cos, sin
import math
from utils.utils import angleToPi
from dataclasses import dataclass
from dataAssociation.hungarian import HungarianAlg
from enum import Enum
import time


STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # landmark srate size [x,y,color]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5   # Number of particle for re-sampling

class ConeColor(Enum):
    BLUE = 0
    YELLOW = 1
    ORANGE = 2
    ORANGE_LARGE = 3
    UNKNOWN = 4

@dataclass
class Particle:

    def __init__(self, N_LM):
        self.weight = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # landmark x-y positions
        self.landmark = np.array([]).reshape(0, LM_SIZE)
        self.landmarkColors = np.array([]).reshape(0, 1)
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))
        self.maxlmID = 0
        self.observations = np.array([])
        

class FastSLAM:
    def __init__(self, numParticles, initPose=np.zeros((1, 3))):
        self.pose = initPose.reshape(-1, 3) # x, y, theta
        self.R = np.diag([0.5, np.deg2rad(5.0)])**2
        self.Q = np.diag([0.05, np.deg2rad(10.0)])**2
        self.numParticles = numParticles
        self.particles = np.array([Particle(100) for _ in range(numParticles)])
        self.maxWeightParticle = self.particles[0]


    def update(self, controlAction, observation, dT = 100e-3):
        '''
        Update the FastSLAM algorithm with the control action and observation

        Parameters
        ----------
        controlAction: np.ndarray, shape=(2,1)
            Control action [v, omega]

        observation: np.ndarray, shape=(3,1)
            Observation of the landmark [distance, bearing, ID], poistion is relative to the car

        Returns
        -------
        particles: List(Particle)
            Updated particles

        maxWeightParticle: Particle
            Particle with the highest weight
        '''

        self.associateObservation(self.particles, observation)
        self.particles = self.predictParticles(self.particles, controlAction, dT)
        self.particles = self.updateWithObservation(self.particles)
        self.particles, _, maxWeightIndex = self.resampling(self.particles)
        self.maxWeightParticle = self.particles[maxWeightIndex]
        return self.particles, self.particles[maxWeightIndex]

    ##########    
    # TESTED #
    ##########
    def associateObservation(self,particles,observation):
        for particle in particles:
            estimateObservation = particle.landmark - np.array([particle.x, particle.y])
            if particle.maxlmID != 0:
                hung = HungarianAlg(observation[:,:2], estimateObservation)
                _, associatedObservations, indeces = hung.solve()
                associatedObservations = np.array(associatedObservations)
                indeces = np.array(indeces)
                ic(indeces.shape)
            else:
                associatedObservations = np.zeros((observation.shape[0],3))
                associatedObservations[:,:2] = observation[:,:2]
                associatedObservations[:,2] = np.arange(observation.shape[0])
                indeces = np.arange(observation.shape[0]).reshape(-1,1)


            particle.observation = np.zeros((associatedObservations.shape[0],4))
            for i in range(associatedObservations.shape[0]):
                compObs = associatedObservations[i,0] + 1j * associatedObservations[i,1]
                dist = np.abs(compObs)
                angle = np.angle(compObs) - particle.yaw
                
                particle.observation[i] = [dist, angle, associatedObservations[i,2], observation[indeces[i,0],2]]
            
        return particles
    
    def predictParticles(self,particles,controlAction,dT):
        '''
        Predict the particles using a simple motion model
        
        Parameters
        ----------
        particles: Particle
            List of particles

        controlAction: np.ndarray, shape=(2,1)
            Control action [v, omega]

        Returns
        -------
        particles: Particle
            Updated particles
        '''
        for i in range(particles.shape[0]):
            noise = (np.random.randn(1, 2)@ self.R) 
            noisyAction = controlAction + noise  # Add noise to the control action
            noisyAction = noisyAction.reshape(-1)
            particles[i] = self.motionModel(particles[i], noisyAction, dT) # Update the particle with the noisy action
        return particles
   
    def motionModel(self,particle,controlAction, dT):
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
        particle.yaw += uTheta
        uX = controlAction[0] * dT * cos(particle.yaw)
        uY = controlAction[0] * dT * sin(particle.yaw)
        particle.x += uX
        particle.y += uY
        return particle
   
    def calcJacobian(self,particle, xf, Pf, Q):
        '''
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
        '''

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
        '''
        Update the Kalman filter

        Parameters
        ----------
        xf: np.ndarray, shape=(2,1)
            position of the landmark

        Pf: np.ndarray, shape=(2,2)
            Covariance of the landmark's position

        v: np.ndarray, shape=(2,1)
            Observation error

        Q: np.ndarray, shape=(2,2)
            Covariance of the measurements

        Hf: np.ndarray, shape=(2,2)
            Jacobian ???

        Returns
        -------
        x: np.ndarray, shape=(2,1)
            Updated position of the landmark

        P: np.ndarray, shape=(2,2)
            Updated Covariance of the landmark's position
        '''
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

    
    def updateWithObservation(self,particles):
        '''
        Update the particles with the observation

        Parameters
        ----------
        particles: List(Particle)
            List of particles

        observation: np.ndarray, shape=(3,1)
            Observation of the landmark [distance, bearing, ID], poistion is relative to the car

        Returns
        -------
        particles: List(Particle)
            Updated particles
        '''
        for indParticle, particle in enumerate(particles):
            for iz in range(particle.observation.shape[0]):
                lmid = int(particle.observation[iz, 2])
                # new landmark
                if lmid >= particle.maxlmID: 
                    particles[indParticle] = self.addNewLandmark(particles[indParticle], particle.observation[iz,:], self.Q)
                    particles[indParticle].maxlmID += 1
                # known landmark
                else:
                    w = self.calcWeightSingle(particles[indParticle], particle.observation[iz, :], self.Q)
                    particles[indParticle].weight *= w
                    particles[indParticle] = self.updateLandmark(particles[indParticle], particle.observation[iz, :], self.Q)
        return particles

    def updateLandmark(self,particle: Particle, observation, Q):
        '''
        Update the landmark position
        
        Parameters
        ----------
        particle: Particle
            The particle to be updated

        observation: np.ndarray, shape=(1,3)
            Observation of the landmark [X, Y, ID], poistion is relative to the car

        Q: np.ndarray, shape=(2,2)
            Covariance of the measurements

        Returns
        -------
        particle: Particle
            Updated particle
        '''

        landmarkID = int(observation[2])
        xf = np.array(particle.landmark[landmarkID, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * landmarkID:2 * landmarkID + 2, :])

        estimateObservation, _, Hf, _ = self.calcJacobian(particle, xf, Pf, Q)

        dz = observation[0:2].reshape(2, 1) - estimateObservation
        dz[1, 0] = angleToPi(dz[1, 0])

        xf, Pf = self.updateKF(xf, Pf, dz, Q, Hf)
        
        particle.landmark[landmarkID, :] = xf.T
        particle.landmarkColors[landmarkID] = observation[3]
        ic(observation,particle.landmarkColors)
        particle.lmP[2 * landmarkID:2 * landmarkID + 2, :] = Pf

        return particle

    def addNewLandmark(self,particle, observation, Q):
        '''
        Add a new landmark to the particle

        Parameters
        ----------
        particle: Particle
            The particle to be updated

        observation: np.ndarray, shape=(3,1)
            Observation of the landmark [X, Y, ID], poistion is relative to the car

        Q: np.ndarray, shape=(2,2)
            Covariance of the measurements

        Returns
        -------
        particle: Particle
            Updated particle
        '''
        r = observation[0]
        b = observation[1]
        # landmarkID = int(observation[2])

        s = math.sin(angleToPi(particle.yaw + b))
        c = math.cos(angleToPi(particle.yaw + b))
        particle.landmark = np.append(particle.landmark, [[particle.x + r * c, particle.y + r * s]], axis=0)
        particle.landmarkColors = np.append(particle.landmarkColors, observation[3])
        # covariance
        Gz = np.array([[c, -r * s],
                    [s, r * c]])

        particle.lmP = np.append(particle.lmP, Gz @ Q @ Gz.T, axis=0)

        return particle
    
    def resampling(self,particles):
        '''
        Resample the particles
        
        Parameters
        ----------
        particles: List(Particle)
            List of particles

        Returns
        -------
        particles: List(Particle)
            Resampled particles
        inds: List(int)
            Indices of the resampled particles
        maxWeightIndex: int
            Index of the particle with the highest weight
        '''
        inds = []
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
