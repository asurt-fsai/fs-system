import py_planning_trees
import numpy as np
import matplotlib.pyplot as plt

cones = [
    [1.85844328, -19.33753632, 3.0, 0.7061382],
    [2.12849187, -17.45331111, 3.0, 0.76776551],
    [2.57394494, -15.60371869, 3.0, 0.74499976],
    [3.19079674, -13.80215222, 3.0, 0.74794124],
    [3.97023274, -12.07025125, 3.0, 0.81150282],
    [4.90804474, -10.41376764, 3.0, 0.82671902],
    [5.66088872, -8.72349226, 3.0, 0.72426541],
    [5.90833204, -6.88947401, 3.0, 0.87854583],
    [5.62830519, -5.06024729, 3.0, 0.84320715],
    [4.84203383, -3.38617496, 3.0, 0.7643888],
    [3.61895913, -1.99763497, 3.0, 0.72241595],
    [2.05360777, -1.01244862, 3.0, 0.88866094],
    [0.2728444, -0.50806865, 3.0, 0.8863489],
    [7.06087551, -18.28892629, 3.0, 0.89996013],
    [7.5407376, -16.47182176, 3.0, 0.73041819],
    [8.23677964, -14.72582811, 3.0, 0.85831636],
    [9.13972114, -13.07675034, 3.0, 0.71911304],
    [10.0160143, -11.3905207, 3.0, 0.76502316],
    [10.60168947, -9.58448286, 3.0, 0.70826409],
    [10.88316488, -7.7071674, 3.0, 0.81210634],
    [10.85028075, -5.80653857, 3.0, 0.81692991],
    [10.50717653, -3.93892093, 3.0, 0.82639325],
    [9.85869466, -2.15350451, 3.0, 0.71564905],
    [8.92780575, -0.50021769, 3.0, 0.70619499],
    [7.73517194, 0.9796313, 3.0, 0.88614058],
    [6.3165248, 2.24270765, 3.0, 0.82546622],
    [4.70903561, 3.2544351, 3.0, 0.79764269],
    [2.95688792, 3.9881141, 3.0, 0.88575413],
    [1.11003903, 4.4238484, 3.0, 0.82517468],
]
cones = np.array(cones)
conesX = cones[:, 0].tolist()
conesY = cones[:, 1].tolist()
conesType = cones[:, 2].astype(int).tolist()
(
    path,
    all_paths,
    waypoints,
    costs,
    detailed_costs,
    left_dists,
    right_dists,
) = py_planning_trees.plan_path(conesX, conesY, conesType)
idx = 0
waypoints = np.array(waypoints)
path = np.array(path)
plt.clf()
plt.scatter(conesX, conesY)
plt.scatter(waypoints[:, 0], waypoints[:, 1])
plt.plot(path[:, 0], path[:, 1])
plt.savefig(f"./images/{idx}_main.png")

for idx2, test_path in enumerate(all_paths[1:]):
    test_path = np.array(test_path)
    plt.clf()
    plt.title(
        "Total: {:.2f}, Path: {:.2f}, Color: {:.2f}, Angle: {:.2f}".format(
            costs[idx2 + 1], *detailed_costs[idx2 + 1]
        )
    )
    if idx in [3, 4]:
        print("Left: ", left_dists[idx2 + 1])
        print("Right: ", right_dists[idx2 + 1])
    plt.scatter(conesX, conesY)
    plt.scatter(waypoints[:, 0], waypoints[:, 1])
    plt.plot(test_path[:, 0], test_path[:, 1])
    plt.savefig(f"./images/{idx}_test_{idx2}.png")
    # if idx2 > 5:
    #     exit()
