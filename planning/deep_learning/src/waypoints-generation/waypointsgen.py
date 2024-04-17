from Datagen import *

import time
import os

count = 0
for j in range(750):
    
    trackGen = TrackGenerator()
    track = trackGen.generate_track(20)
    smoothed = trackGen.smooth_track(*track, 5)
    spline = trackGen.fit_spline(*smoothed, 300)
    cones_data = trackGen.generate_cones(*spline, track_width=2, distance_between_cones=10)

    dataGen = DataGenerator(cones_data, spline)

    src,tgt = dataGen.generate_data()
    src = dataGen.oneHotEncode(src)
    src,tgt = dataGen.ToTensors(src,tgt)
    plt.figure()
    trackGen.plot_track(spline[0], spline[1])
    
    src_file_path = f"inputs/tensorsV2/inputs/train/src{j}.pt"
    tgt_file_path = f"inputs/tensorsV2/outputs/train/tgt{j}.pt"
    
    for i in cones_data:
        trackGen.plot_cones(cones_data[i][0], cones_data[i][1], color=i)

    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis
    plt.show(block = False)
    plt.pause(1)
    plt.close()
    
    while os.path.exists(src_file_path) or os.path.exists(tgt_file_path):
        j += 1
        src_file_path = f"inputs/tensorsV2/inputs/train/src{j}.pt"
        tgt_file_path = f"inputs/tensorsV2/outputs/train/tgt{j}.pt"
    
    torch.save(src, src_file_path)
    torch.save(tgt, tgt_file_path)
    print("count: ", count+1)
    count += 1
print("Data saved to src.pt and tgt.pt")