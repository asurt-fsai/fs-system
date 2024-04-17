from Datagen import *

import time
import os


for j in range(100):
    
    trackGen = TrackGenerator()
    track = trackGen.generate_track(28)
    smoothed = trackGen.smooth_track(*track, 5)
    spline = trackGen.fit_spline(*smoothed, 30)
    cones_data = trackGen.generate_cones(*spline, track_width=4, distance_between_cones=4)

    dataGen = DataGenerator(cones_data, spline)

    src,tgt = dataGen.generate_data()
    src = dataGen.oneHotEncode(src)
    src,tgt = dataGen.ToTensors(src,tgt)
    plt.figure()
    trackGen.plot_track(spline[0], spline[1])
    
    src_file_path = f"inputs/tensorsV2/inputs/src{j}.pt"
    tgt_file_path = f"inputs/tensorsV2/outputs/tgt{j}.pt"
    
    for i in cones_data:
        trackGen.plot_cones(cones_data[i][0], cones_data[i][1], color=i)

    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis
    plt.show(block = False)
    plt.pause(3)
    plt.close()
    
    while os.path.exists(src_file_path) or os.path.exists(tgt_file_path):
        j += 1
        src_file_path = f"inputs/tensorsV2/inputs/src{j}.pt"
        tgt_file_path = f"inputs/tensorsV2/outputs/tgt{j}.pt"
    
    torch.save(src, f"inputs/tensorsV2/inputs/src{j}.pt")
    torch.save(tgt, f"inputs/tensorsV2/outputs/tgt{j}.pt")
print("Data saved to src.pt and tgt.pt")