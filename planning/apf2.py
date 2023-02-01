import math
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
#from .consts import *

class Vector2d:
    """
    A class to represent a 2-dimensional vector.

    Attributes:
        deltax (float): The x-coordinate of the vector.
        deltay (float): The y-coordinate of the vector.
        length (float): The magnitude of the vector.
        direction (list): The direction of the vector represented as a unit vector.

    Methods:
        vector2d_share: A method to calculate the magnitude and direction
        of the vector.
        __add__: Overloads the addition operator for vectors.
        __sub__: Overloads the subtraction operator for vectors.
        __mul__: Overloads the multiplication operator for vectors.
        __truediv__: Overloads the true division operator for vectors.
    """

    def __init__(self, xposition, yposition):

        self.deltax = xposition
        self.deltay = yposition
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
  
        """
        This method updates the instance variables `length` and `direction` of the `Vector2d` class,
        based on the values of `deltax` and `deltay`. If `deltax` and `deltay` are both lists,
        then the difference between their elements is calculated and stored in `deltax` 
        and `deltay`.The `length` is then calculated as the square root of the sum of the squares of `deltax` and `deltay`.
        If the `length` is positive, the `direction` is calculated as the division of `deltax` and `deltay` by the `length`, otherwise the `direction` is set to `None`.
        Returns:
         None
        """
        if isinstance(self.deltax,list) and isinstance(self.deltay,list):
            deltax, deltay = self.deltax, self.deltay
            self.deltax = deltay[0] - deltax[0]
            self.deltay = deltay[1] - deltax[1]
            self.length = math.sqrt(self.deltax**2 + self.deltay**2) * 1.0
            if self.length > 0:
                self.direction = [self.deltax / self.length, self.deltay / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltax**2 + self.deltay**2) * 1.0
            if self.length > 0:
                self.direction = [self.deltax / self.length, self.deltay / self.length]
            else:
                self.direction = None

    def __add__(self, other):

        """
        This method overloads the '+' operator to add two `Vector2d` objects.
        The '+' operator returns a new `Vector2d` object which is the result of
        adding the `deltax` and `deltay` of the two input `Vector2d` objects.

        Parameters
        ----------
        other : Vector2d
            The second `Vector2d` object to be added.

        Returns
        -------
        Vector2d
            A new `Vector2d` object which is the result of adding the `deltax` and `deltay`
            of the two input `Vector2d` objects.
        """

        vec = Vector2d(self.deltax, self.deltay)
        vec.deltax += other.deltax
        vec.deltay += other.deltay
        vec.vector2d_share()
        return vec

    def __sub__(self, other):

        """
        This method overloads the '-' operator to subtract two `Vector2d` objects.
        The '-' operator returns a new `Vector2d` object which is the result of
        subtracting the `deltax` and `deltay` of the second `Vector2d` object
        from the first `Vector2d` object.

        Parameters
        ----------
        other : Vector2d
            The second `Vector2d` object to be subtracted.

        Returns
        -------
        Vector2d
            A new `Vector2d` object which is the result of subtracting the `deltax` and `deltay`
            of the second `Vector2d` object from the first `Vector2d` object.
        """
        vec = Vector2d(self.deltax, self.deltay)
        vec.deltax -= other.deltax
        vec.deltay -= other.deltay
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        """
        This method overloads the '*' operator to multiply a `Vector2d` object by
        a scalar.
        The '*' operator returns a new `Vector2d` object which is the result of
        multiplying the `deltax` and `deltay` of the `Vector2d` object by the
        scalar.

        Parameters
        ----------
        other : int or float
            The scalar to be multiplied with the `Vector2d` object.

        Returns
        -------
        Vector2d
            A new `Vector2d` object which is the result of multiplying the `deltax`
            and `deltay` of the `Vector2d` object by the scalar.
        """
        vec = Vector2d(self.deltax, self.deltay)
        vec.deltax *= other
        vec.deltay *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        """
        This method overloads the '/' operator to divide a `Vector2d` object by a scalar.
        The '/' operator returns a new `Vector2d` object which is the result of
        dividing the `deltax` and `deltay` of the `Vector2d` object by the scalar.

        Parameters
        ----------
        other : int or float
            The scalar to divide the `Vector2d` object with.

        Returns
        -------
        Vector2d
            A new `Vector2d` object which is the result of dividing the `deltax`
            and `deltay`
            of the `Vector2d` object by the scalar.
        """
        return self.__mul__(1.0 / other)


class APF:
    """
    The APF class implements the Artificial Potential Field algorithm.

    Parameters of constructor
    ----------
    start : tuple
        The start point in the form of (x, y).
    goal : tuple
        The goal point in the form of (x, y).
    obstacles : list
        A list of obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    k_attractive : float
        The attractive force coefficient.
    k_repulsive : float
        The repulsive force coefficient.
    p_node : float
        The radius around an obstacle within which the repulsive force is active.
    step_size : float
        The step size for moving from the current position to the next position.
    max_iterations : int
        The maximum number of iterations for the algorithm.
    goal_threshold : float
        The threshold distance from the goal at which the algorithm is considered successful.

    Attributes
    ----------
    start : Vector2d
    current_position : Vector2d
    goal : Vector2d
    obstacles : list of Vector2d
    k_attractive : float
    k_repulsive : float
    p_node : float
    step_size : float
    max_iterations : int
    iterations : int
        The current iteration number.
    goal_threshold : float
    path : list
        The path computed by the algorithm, in the form of a list of (x, y) tuples.
    is_path_plan_success : bool
        A flag indicating whether the algorithm was successful or not.
    delta_t : float
        The time interval for pausing the plot during the algorithm.

    Methods
    -------
    attractive()
        Computes the attractive force vector.
    repulsion()
        Computes the repulsive force vector.
    path_plan()
        Implements the path planning algorithm using the Artificial Potential Field method.
    """

    def __init__(
        self,
        start: (),
        goal: (),
        obstacles: [],
        k_attractive: float,
        k_repulsive: float,
        p_node: float,
        step_size: float,
        max_iterations: int,
        goal_threshold: float,
    ):

        self.start = Vector2d(start[0], start[1])
        self.current_position = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_attractive = k_attractive
        self.k_repulsive = k_repulsive
        self.p_node = p_node
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.iterations = 0
        self.goal_threshold = goal_threshold
        self.path = [] 
        self.is_path_plan_success = False
        self.delta_t = 0.01

    def attractive(self):
        """
        Calculates the attractive force acting on the current position.

        Returns:
            att (Vector2d): The attractive force vector.
        """
        att = (self.goal - self.current_position) * self.k_attractive
        return att

    def repulsion(self):
        """
        Calculates the repulsive force acting on the current position for each obstacle.

        Returns:
            rep (Vector2d): The repulsive force vector.
        """
        rep = Vector2d(0, 0)
        for obstacle in self.obstacles:

            distance_to_obstacle = self.current_position - obstacle  # distance to obstacle
            if distance_to_obstacle.length > self.p_node:
                #If the distance between the object and the 
                #obstacle is greater than p_node, the function skips to the next obstacle. 
                #This means that the repulsive force is only calculated for obstacles that are 
                #close enough to have an effect.
               pass
            else:
                rep1 = (
                    Vector2d(distance_to_obstacle.direction[0], distance_to_obstacle.direction[1])
                    * self.k_repulsive
                    * (1.0 / distance_to_obstacle.length - 1.0 / self.p_node)
                    / (distance_to_obstacle.length**2)
                )
                rep2 = Vector2d(distance_to_obstacle.direction[0], distance_to_obstacle.direction[1]) * self.k_repulsive * ((1.0 / distance_to_obstacle.length - 1.0 / self.p_node) ** 2) * distance_to_obstacle.length
                rep +=(rep1+rep2)
        return rep

    def path_plan(self):
    
        """
        This function implements the potential field algorithm. It calculates the attractive and
        repulsive forces acting on the current position and updates the position of the robot
        iteratively. The algorithm continues until the goal is reached or the maximum number of iterations is reached.
        The path taken by the robot is stored in the `path` list. The function also updates the value of the `is_path_plan_success`
        flag to indicate if the path planning was successful and a path is found.

        Returns:
            None
        """

        while (
            self.iterations < self.max_iterations
            and (self.current_position - self.goal).length > self.goal_threshold
        ):
            f_vec = self.attractive() + self.repulsion()
            self.current_position += (
                Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            )
            self.iterations += 1
            self.path.append([self.current_position.deltax, self.current_position.deltay])

            plt.plot(self.current_position.deltax, self.current_position.deltay, ".b")
            plt.pause(self.delta_t)
        if (self.current_position - self.goal).length <= self.goal_threshold:
            self.is_path_plan_success = True


if __name__ == "__main__":
    # Constants used in the algorithm
    # k_attractive = 1.0  # attractive force constant
    # k_repulsive = 100.0  # repulsive force constant
    # p_node = 3  # radius of obstacles
    # step_size = 0.2  # step size used for updating position
    # max_iterations = 500  # maximum number of iterations
    # goal_threshold = 0.2  # threshold for stopping the algorithm
    

    # Start and goal positions
    start = (0, 0)
    goal = (15, 15)

    # Plot the start and goal positions on the graph
    fig = plt.figure(figsize=(7, 7))
    subplot = fig.add_subplot(111)
    subplot.set_xlabel("X-distance: m")
    subplot.set_ylabel("Y-distance: m")
    subplot.plot(start[0], start[1], "*r")
    subplot.plot(goal[0], goal[1], "*r")

    # List of obstacles positions on the graph
    obs = [
        [1, 4],
        [2, 4],
        [3, 3],
        [6, 1],
        [6, 7],
        [10, 6],
        [11, 12],
        [14, 14],
        [12.5, 7.5],
        [17.5, 12.5],
        [7.5, 10.0],
    ]
    #obstacle at 17.5 and 12.5 causes local minima 

    # Plot the obstacles on the graph
    for OB in obs:
        circle = Circle(xy=(OB[0], OB[1]), radius=3, alpha=0.3)
        subplot.add_patch(circle)
        subplot.plot(OB[0], OB[1], "xk")

    # Create an instance of the APF class
    apf = APF(
        (0,0),
        (15,15),
        obs,
        1.0,
        60,
        2,
        0.2,
        500,
        0.2,
    )

    # Run the algorithm
    apf.path_plan()

    # If path planning is successful, plot the path on the graph
    if apf.is_path_plan_success:
        path = apf.path
        plt.show()
    else:
        # If path planning is not successful, print an error message
        print("path plan failed")
