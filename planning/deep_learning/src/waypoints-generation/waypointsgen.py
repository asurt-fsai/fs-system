'''importing the necessary libraries for generating waypoints'''
import os
import torch
from matplotlib import plt
from Trackgen import TrackGenerator
from Datagen import DataGenerator


COUNT = 0
for j in range(750):

    trackGen = TrackGenerator()
    track = trackGen.generateTrack(20)
    smoothed = trackGen.smoothTrack(track, 5)
    spline = trackGen.fitSpline(smoothed, 300)
    conesData = trackGen.generateCones(spline[0], spline[1], trackWidth=2, distanceBetweenCones=10)
    dataGen = DataGenerator(conesData, spline)

    src,tgt = dataGen.generateData()
    src = dataGen.oneHotEncode(src)
    src,tgt = dataGen.toTensors(src,tgt)
    plt.figure()
    trackGen.plotTrack(spline[0], spline[1])
    src_file_path = f"inputs/tensorsV2/inputs/train/src{j}.pt"
    tgt_file_path = f"inputs/tensorsV2/outputs/train/tgt{j}.pt"

    for i, cones in conesData.items():
        trackGen.plotCones(cones[0], cones[1], color=i)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block = False)
    plt.pause(1)
    plt.close()

    while os.path.exists(src_file_path) or os.path.exists(tgt_file_path):
        j += 1
        src_file_path = f"inputs/tensorsV2/inputs/train/src{j}.pt"
        tgt_file_path = f"inputs/tensorsV2/outputs/train/tgt{j}.pt"

    torch.save(src, src_file_path)
    torch.save(tgt, tgt_file_path)
    print("count: ", COUNT+1)
    COUNT += 1
print("Data saved to src.pt and tgt.pt")
