#!/usr/bin/python3
"""
.

"""
import rospy
from supervisor.helpers import Module  # type: ignore[attr-defined]
from supervisor.helpers import Visualizer  # type: ignore[attr-defined]


def main() -> None:
    """
    Main function.
    """
    rospy.init_node("visualizer_test_node")
    visualizer = Visualizer("/visualizer/text", "/visualizer/buttons")
    module = Module("supervisor", "placeholder_test_node.launch", "/status/placeholder_test_node")
    module.launch()
    visualizer.addButton(5, 0, module)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer.visualizeText(
            [[0, 0, "Hello World", "w", "map", 0], [2.5, 0, f"{module.rate:.2f}hz", "w", "map", 0]]
        )
        rate.sleep()

    module.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
