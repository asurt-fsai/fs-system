#!/usr/bin/python3
"""
Main ROS node for publishing skidpad waypoints
"""


import rospy
from visualization_msgs.msg import Marker, MarkerArray
from skidpad.skidpad import Skidpad


class ConesNode:  # pylint: disable=R0903, R0914
    """
    ROS node for publishing skidpad waypoints.
    """

    def run(self) -> None:
        """
        Run the cone positions node.

        Parameters:
        ------------
        None

        Returns:
        -----------
        None
        """
        rospy.init_node("cone_positions_node")
        publisher = rospy.Publisher("/cone_positions_topic", MarkerArray, queue_size=10)
        rate = rospy.Rate(10)
        distance = rospy.get_param("/cone_positions_node/distance")
        innerRadius = rospy.get_param("/cone_positions_node/inner_radius")
        outerRadius = rospy.get_param("/cone_positions_node/outer_radius")
        lengthOfLineT = rospy.get_param("/cone_positions_node/len_of_line_t")
        lengthOfLineB = rospy.get_param("/cone_positions_node/len_of_line_b")
        step = rospy.get_param("/cone_positions_node/step")

        skidpad = Skidpad(distance, innerRadius, outerRadius, lengthOfLineB, lengthOfLineT)
        conesPosition = skidpad.getConesPosition(step)

        while not rospy.is_shutdown():
            markerArray = MarkerArray()

            for i, position in enumerate(conesPosition):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                markerArray.markers.append(marker)

            # Publish the MarkerArray
            publisher.publish(markerArray)

            rate.sleep()


if __name__ == "__main__":
    node = ConesNode()
    node.run()
