#! /usr/bin/env python3
"""
Node to publish the car mesh for visualization in RViz
"""
import os
import rospy
import rospkg
from visualization_msgs.msg import MarkerArray, Marker


def main() -> None:
    """
    Main function
    """
    rospy.init_node("mesh_visualizer")

    rosPack = rospkg.RosPack()
    vizPath = os.path.join(rosPack.get_path("supervisor"), "meshes")
    files = [file for file in os.listdir(vizPath) if file.endswith((".dae", ".stl", ".mesh"))]

    markerPub = rospy.Publisher("/visualizer/meshes", MarkerArray, queue_size=1, latch=True)

    markerArray = MarkerArray()
    for markerId, file in enumerate(files):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = markerId
        marker.mesh_resource = "package://supervisor/meshes/" + file
        marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        marker.type = marker.MESH_RESOURCE
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.orientation.w = 1
        markerArray.markers.append(marker)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        markerPub.publish(markerArray)
        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
