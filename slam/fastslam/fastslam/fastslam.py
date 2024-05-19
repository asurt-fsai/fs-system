import numpy as np
from .hungarian import HungarianAlg

# import types for numpy arrays
from typing import List
from math import cos, sin
import math

Q = np.diag([3.0, np.deg2rad(10.0)])**2
R = np.diag([1.0, 1.0, np.deg2rad(20.0)])**2

STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

class Particle:

    def __init__(self, N_LM):
        self.weight = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # landmark x-y positions
        self.lm = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))
        self.maxlmID = 0

class FastSLAM:
    def __init__(self, initPose=np.zeros((1, 3))):
        self.pose = initPose.reshape(-1, 3) # x, y, theta
        self.dT = 100e-3
        self.R = np.diag([1.0, np.deg2rad(20.0)])**2
        self.Q = np.diag([3.0, np.deg2rad(10.0)])**2
        


    def predict_particles(self,particles):
        for i in range(particles.shape[0]):
            px = np.zeros((3, 1))
            px[0, 0] = particles[i].x
            px[1, 0] = particles[i].y
            px[2, 0] = particles[i].yaw
            px = px + (np.random.randn(1, 3) @ self.R).T  # add noise
            particles[i].x = px[0, 0]
            particles[i].y = px[1, 0]
            particles[i].yaw = px[2, 0]

        return particles

    def pi_2_pi(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
   


    def update_with_observation(self,particles, z):
        for iz in range(len(z[0, :])):

            lmid = int(z[2, iz])

            for indParticle in range(self.numParticles):
                # new landmark
                if abs(particles[indParticle].lm[lmid, 0]) <= 0.01:
                    particles[indParticle] = self.add_new_lm(particles[indParticle], z[:, iz], self.Q)
                # known landmark
                else:
                    w = self.compute_weight(particles[indParticle], z[:, iz], self.Q)
                    particles[indParticle].weight *= w
                    particles[indParticle] = self.update_landmark(particles[indParticle], z[:, iz], self.Q)

        return particles
        
    def compute_weight(self,particle, z, Q):
        lm_id = int(z[2])
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf, Q)
        dx = z[0:2].reshape(2, 1) - zp
        dx[1, 0] = self.pi_2_pi(dx[1, 0])

        try:
            invS = np.linalg.inv(Sf)
        except np.linalg.linalg.LinAlgError:
            print("singuler")
            return 1.0

        num = math.exp(-0.5 * dx.T @ invS @ dx)
        den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))
        w = num / den

        return w

    def compute_jacobians(self,particle, xf, Pf, Q):
        dx = xf[0, 0] - particle.x
        dy = xf[1, 0] - particle.y
        d2 = dx**2 + dy**2
        d = math.sqrt(d2)

        zp = np.array(
            [d, self.pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

        Hv = np.array([[-dx / d, -dy / d, 0.0],
                    [dy / d2, -dx / d2, -1.0]])

        Hf = np.array([[dx / d, dy / d],
                    [-dy / d2, dx / d2]])

        Sf = Hf @ Pf @ Hf.T + Q

        return zp, Hv, Hf, Sf

    def add_new_lm(self,particle, z, Q):

        r = z[0]
        b = z[1]
        lm_id = int(z[2])

        s = math.sin(self.pi_2_pi(particle.yaw + b))
        c = math.cos(self.pi_2_pi(particle.yaw + b))

        particle.lm[lm_id, 0] = particle.x + r * c
        particle.lm[lm_id, 1] = particle.y + r * s

        # covariance
        Gz = np.array([[c, -r * s],
                    [s, r * c]])

        particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ Q @ Gz.T

        return particle

    def update_KF_with_cholesky(xf, Pf, v, Q, Hf):
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

    def update_landmark(self,particle, z, Q):

        lm_id = int(z[2])
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2, :])

        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf, Q)

        dz = z[0:2].reshape(2, 1) - zp
        dz[1, 0] = self.pi_2_pi(dz[1, 0])

        xf, Pf = self.update_KF_with_cholesky(xf, Pf, dz, Q, Hf)

        particle.lm[lm_id, :] = xf.T
        particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

        return particle
        
    def normalize_weight(self,particles):

        sumWeight = sum([p.weight for p in particles])

        try:
            for i in range(self.numParticles):
                particles[i].weight /= sumWeight
        except ZeroDivisionError:
            for i in range(self.numParticles):
                particles[i].weight = 1.0 / self.numParticles

            return particles

        return particles


    def resampling(self,particles):
        """
        low variance re-sampling
        """

        particles = self.normalize_weight(particles)
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