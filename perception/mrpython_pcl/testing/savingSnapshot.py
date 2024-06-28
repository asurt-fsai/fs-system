"""
Script used to extract a point cloud from a bag and save it as a .npy file
"""
import rosbag
import ros_numpy
import numpy as np
import sensor_msgs


def main() -> None:
    """
    Run the script
    """
    bag = rosbag.Bag("./testing/double_lap_processed.bag")
    startTime = None
    for topic, msg, curTime in bag.read_messages():
        if topic == "/velodyne_points":
            if startTime is None:
                startTime = curTime.to_sec()
            curTime = curTime.to_sec() - startTime
            if curTime > 2:
                # pylint: disable=protected-access
                msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
                toSave = ros_numpy.numpify(msg)
                x = toSave["x"]
                y = toSave["y"]
                z = toSave["z"]
                intensity = toSave["intensity"]

                cloudArr = np.hstack(
                    (x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1), intensity.reshape(-1, 1))
                )
                np.save("./testing/pointcloud_1.npy", cloudArr)
                break
    bag.close()


if __name__ == "__main__":
    main()
