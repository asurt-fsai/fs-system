import numpy as np
from icecream import ic

import time

class HungarianAlg(object):
    def __init__(self, observationMatrix, landmarkMatrix):
        """
        This creates a HungarianAlg object with the cost matrix associated to it. It stores a copy of the matrix as well as the original.
        It then records the shape and initiates some helper variables, like the covers for the rows and columns and the markings.
        """
        self.debug=False
        self.obsIDs = []
        self.cleanedObsIDs = []
        self.threshold = 2.5
        self.observationMatrix = observationMatrix
        self.landmarkMatrix = landmarkMatrix
        self.removedObservations = []

        self.orgCostMatrix = self.calcCostMatrix()
        self.orgCostMatrix = self.removeFarCones()
        self.O = self.orgCostMatrix
        self.C = self.orgCostMatrix
        self.solTrialNum = 0
        self.maxSolTrialNum = 5
        try:
            self.n, self.m = self.C.shape
        except:
            self.n = self.C.shape
            self.m = 0
        self.rowCovered = np.zeros(self.n, dtype=bool)
        self.colCovered = np.zeros(self.m, dtype=bool)
        self.marked = np.zeros((self.n, self.m), dtype=int)
        self.solution = []

    def calcCostMatrix(self):
        # Calculate the cost matrix (distance between observations and landmarks)
        # costMatrix = np.zeros((self.observationMatrix.shape[0], self.landmarkMatrix.shape[0]))
        # for i in range(self.observationMatrix.shape[0]):
        #     self.obsIDs.append(i)  # Store the observation IDs
        #     for j in range(self.landmarkMatrix.shape[0]):
        #         costMatrix[i, j] = np.linalg.norm(
        #             self.observationMatrix[i] - self.landmarkMatrix[j]
        #         )  # Euclidean distance
        # costMatrix = costMatrix.reshape(
        #     self.observationMatrix.shape[0], self.landmarkMatrix.shape[0]
        # )
        # Code above is slow code below is 100x faster
        # convert obs and landmarks to complex !!! 200IQ MOVE !!!
        obs = self.observationMatrix[:, 0] + 1j * self.observationMatrix[:, 1]
        landmarks = self.landmarkMatrix[:, 0] + 1j * self.landmarkMatrix[:, 1]
        # calculate the cost matrix
        costMatrix = np.abs(obs[:, np.newaxis] - landmarks) # IM FAST AF BOIIII
        return costMatrix

    def removeFarCones(self):
        # Remove observations that are too far from any landmark
        cleanedCostMatrix = np.array([])
        for i in range(self.orgCostMatrix.shape[0]):
            if np.min(self.orgCostMatrix[i]) > self.threshold:
                self.removedObservations.append(i)  # Store the removed observation IDs
            else:
                cleanedCostMatrix = np.append(cleanedCostMatrix, self.orgCostMatrix[i], axis=0)
                self.cleanedObsIDs.append(i)
        cleanedCostMatrix = cleanedCostMatrix.reshape(
            len(self.cleanedObsIDs), self.orgCostMatrix.shape[1]
        )
        return cleanedCostMatrix

    def clearCovers(self):
        """
        This clears any covers, as they can change meaning from one step to another
        """
        self.rowCovered[:] = False
        self.colCovered[:] = False

    def clearMarks(self):
        """
        Clears marks when trying new solutions
        """
        self.marked[:, :] = 0

    def solve(self):
        """
        This chooses an initial step for the process and then begins following the appropriate steps.
        It saves the assignment solution to self.solution and the final cost found to self.minCost.
        """
        initStep = step0
        if self.n == self.m:
            initStep = step1

        step = initStep

        while type(step) is not tuple:
            step = step(self)

        if step[0]:
            self.solution = step[2]
            self.minCost = step[1]

            # Assign the removed rows to new landmarks
            noLandmarks = self.orgCostMatrix.shape[1]
            for i in range(len(self.removedObservations)):
                self.solution.append([self.removedObservations[i], noLandmarks + i])
                self.landmarkMatrix = np.append(
                    self.landmarkMatrix,
                    self.observationMatrix[self.removedObservations[i]].reshape(1, 2),
                    axis=0,
                )

            associatedObservations = []
            for i in range(len(self.solution)):
                associatedObservations.append(np.hstack((self.observationMatrix[self.solution[i][0]],self.solution[i][1])))
        # self.printResults()
        return self.landmarkMatrix, associatedObservations, self.solution

    def printResults(self):
        if self.solution == None:
            raise Exception(
                "No solution was computed yet or there is no solution. Run the solve method or try another cost matrix."
            )
        for i in range(len(self.solution)):
            print(
                "Observation {} was assigned to landmark {}".format(
                    self.observationMatrix[self.solution[i][0]],
                    self.landmarkMatrix[self.solution[i][1]],
                )
            )


def step0(state):
    start = time.perf_counter()
    """
    This step pads the matrix so that it's squared
    """
    matrixSize = max(state.n, state.m)
    colPad = matrixSize - state.n
    rowPad = matrixSize - state.m
    state.C = np.pad(state.C, ((0, colPad), (0, rowPad)), "constant", constant_values=(0))
    state.rowCovered = np.zeros(state.C.shape[0], dtype=bool)
    state.colCovered = np.zeros(state.C.shape[1], dtype=bool)
    state.marked = np.zeros((state.C.shape[0], state.C.shape[1]), dtype=int)
    if state.debug:
        print("Step 0 took: ", time.perf_counter() - start)
    return step1


def step1(state):
    """
    Subtracts the minimum value per column for each cell of that column
    """
    start = time.perf_counter()
    state.C = state.C - np.min(state.C, axis=1)[:, np.newaxis]
    if state.debug:
        print("Step 1 took: ", time.perf_counter() - start)
    return step2


def step2(state):
    """
    Subtracts the minimum value per row for each cell of that row
    """
    start = time.perf_counter()
    state.C = state.C - np.min(state.C, axis=0)[np.newaxis, :]
    if state.debug:
        print("Step 2 took: ", time.perf_counter() - start)
    return step3


def step3(state):
    """
    This step tries to find a coverage of all zeroes in the matrix using the minimum amount of row/column covers.
    It then uses this coverage to check for a solution. If one is found, the algorithm stops. Otherwise, it goes to step 4 and back to step 3.
    """
    start = time.perf_counter()
    rowMarked = np.zeros(state.C.shape[0], dtype=bool)
    colMarked = np.zeros(state.C.shape[1], dtype=bool)

    for j in range(state.C.shape[1]):
        for i in range(state.C.shape[0]):
            if not state.rowCovered[i] and not state.colCovered[j] and state.C[i][j] == 0:
                state.marked[i][j] = 1
                state.rowCovered[i] = True
                state.colCovered[j] = True

    state.clearCovers()

    for i in range(state.C.shape[0]):
        if np.sum(state.marked[i, :]) == 0:
            rowMarked[i] = True
            for j in range(state.C.shape[1]):
                if not colMarked[j] and state.C[i][j] == 0:
                    colMarked[j] = True
                    for k in range(state.C.shape[0]):
                        if not rowMarked[k] and state.marked[k][j] == 1:
                            rowMarked[k] = True

    state.rowCovered = np.logical_not(rowMarked)
    state.colCovered = colMarked
    numLines = np.sum(state.rowCovered) + np.sum(state.colCovered)
    if state.debug:
        print("Step 3 took: ", time.perf_counter() - start)
    if numLines == state.C.shape[0]:
        sol = checkSol(state)
        return sol
    else:
        if state.solTrialNum >= state.maxSolTrialNum:
            return False, 0, []
        state.solTrialNum += 1 # fix for infinite loop
        return step4


def step4(state):
    """
    If no solution was found in step 3, this step changes some values in the matrix so that we may now find some coverage.
    The algorithm may be stuck in a step 3 - step 4 loop. If it happens, there is no solution or the wrong matrix was given.
    """
    start = time.perf_counter()
    minUncovered = np.inf
    for i in range(state.C.shape[0]):
        for j in range(state.C.shape[1]):
            if not state.rowCovered[i] and not state.colCovered[j] and state.C[i][j] < minUncovered:
                minUncovered = state.C[i][j]

    for i in range(state.C.shape[0]):
        for j in range(state.C.shape[1]):
            if not state.rowCovered[i] and not state.colCovered[j]:
                state.C[i][j] -= minUncovered
            elif state.rowCovered[i] and state.colCovered[j]:
                state.C[i][j] += minUncovered

    state.clearCovers()
    state.clearMarks()
    if state.debug:
        print("Step 4 took: ", time.perf_counter() - start)
    return step3


def checkSol(state):
    """
    This method uses the coverage of the cost matrix to try and find a solution.
    """
    start = time.perf_counter()
    for j in range(state.C.shape[1]):
        for i in range(state.C.shape[0]):
            if not state.rowCovered[i] and not state.colCovered[j] and state.C[i][j] == 0:
                state.marked[i][j] = 1
                state.rowCovered[i] = True
                state.colCovered[j] = True
    sol = []
    cost = 0

    for i in range(state.n):
        for j in range(state.m):
            if state.marked[i][j] == 1 and i < len(state.cleanedObsIDs):
                sol.append([state.cleanedObsIDs[i], j])
                cost = cost + state.O[i][j]

    state.clearCovers()
    if state.debug:
        print("CheckSol took: ", time.perf_counter() - start)   
    return len(sol) == state.orgCostMatrix.shape[0], cost, sol
