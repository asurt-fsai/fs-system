"""Main file to run the adaptive pure pursuit controller."""
import numpy as np
import rclpy
from kinematic_bicycle import Car


def main() -> None:
    """Main function to initialize the node and the car object."""
    rclpy.init()
    car = Car(np.array([0, 0, 0.2, 0]), 0.1, 2.5)  # initial state of the car
    rclpy.spin(car)
    car.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
