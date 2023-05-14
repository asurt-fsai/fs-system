from planning_apf import APF
from planning_sim_tester import SimConfig
from planning_sim_tester import Simulator
from planning_sim_tester import Track, TrackConfig
import numpy as np
import matplotlib.pyplot as plt
import time

final_cones = None

istest = False


class PlannerWrapper:
    def __init__(self):
        self.counter = 0

    def getPath(self, cones, verbose=False):
        global final_cones
        final_cones = cones
        # print("planning")
        print(self.counter)

        blue_cones = []
        yellow_cones = []
        all_cones = []
        for cone in cones:
            if cone[2] == 0:
                blue_cones.append(cone[:2].tolist())
            elif cone[2] == 1:
                yellow_cones.append(cone[:2].tolist())
            all_cones.append(cone[:2].tolist())

        apfTest = APF(
            (0, 0),
            (0, 0),
            all_cones,
            3.5,
            30,
            1.3,
            0.2,
            25,
            0.2,
            yellow_cones,
            blue_cones,
            True,
        )

        apfTest.pathPlanPlot()
        path = np.array(apfTest.path)

        self.counter += 1
        return path


if istest:
    cones = np.load("failed_cones.npy")
    planner_wrapper = PlannerWrapper()
    for i in range(10):
        planner_wrapper.getPath(cones, True)
else:
    crns = np.array(
        [False, True, True, False, True, True, True, False, True, True, False],
        dtype=bool,
    )
    delTh = np.array(
        [
            0,
            np.pi / 2,
            np.pi / 2,
            0,
            np.pi / 2,
            np.pi / 2,
            np.pi / 2,
            0,
            np.pi / 4,
            np.pi / 4,
            0,
        ],
        dtype=float,
    )
    lpar = np.array([20, 10, -10, 20, 10, -10, 10, 200, -10, 10, 200], dtype=float)
    track = Track(TrackConfig(), crns)
    track.solve(lpar, delTh, case=0)
    aveDist = 2
    track.plot(cones=True, aveDist=aveDist)

    simulator = Simulator(SimConfig(), track)
    out = simulator.run(PlannerWrapper)
    planner_wrapper = PlannerWrapper()
    planner_wrapper.getPath(final_cones, True)
    np.save("failed_cones.npy", final_cones)
