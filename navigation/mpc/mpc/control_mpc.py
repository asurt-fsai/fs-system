"""
this module is used to control the vehicle using the MPC controller
"""
import math
import do_mpc
import numpy as np
import rclpy
from casadi import SX, MX, tan
from casadi import SXStruct, MXStruct
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from tf_transformations import euler_from_quaternion


class LinearkinamaticMPC(Node):  # type: ignore[misc] # pylint: disable=too-many-instance-attributes
    """
    This class is the MPC controller for the vehicle control
    The class is used to control the vehicle using the MPC controller
    The class has the following methods:
    __init__: initialize the MPC controller
    stateCallback: callback function for the state of the vehicle
    pathCallback: callback function for the path of the vehicle
    modelConfig: Model configuration for the vehicle kinematics
    mpcConfigure: MPC configuration for the vehicle control
    costFunctionConfigure: configure the cost function of the MPC controller
    constrainMPC: set the constraints of the MPC controller
    tvpFun: time-varying parameters function for the MPC controller
    mpcFinalSetup: Final setup of the MPC controller
    ecludian: calculate the ecludian distance between two points
    mpcSimulator: MPC simulator for the vehicle control
    run: The function runs the MPC controller for the vehicle control
    """

    def __init__(self, lFront: float, lRear: float, nHorizon: float, tStep: float) -> None:
        super().__init__("MPC_NODE")
        ##Define nodes and Publisher and subscriber
        rclpy.init(args=None)
        self.mpcNode = rclpy.create_node("MPC_NODE")
        self.controlPub = self.mpcNode.create_publisher(AckermannDrive, "/control", 10)
        self.stateSub = self.mpcNode.create_subscription(Odometry, "/state", self.stateCallback, 10)
        self.pathSub = self.mpcNode.create_subscription(Path, "/path", self.pathCallback, 10)
        self.waypoints = Path()  # initialize the waypoints
        self.state = Odometry()  # initialize the state
        self.controlMsg = AckermannDrive()  # initialize the control
        # initalize Controller Variable
        self.nHorizon: float = nHorizon
        self.tStep: float = tStep
        # Initate the model and it's configurations
        modelType = "continuous"
        self.model = do_mpc.model.Model(modelType)
        # define the input of the model
        self.vel: SX | MX = self.model.set_variable(
            var_type="_u", var_name="velocity", shape=(1, 1)
        )
        self.steering: SX | MX = self.model.set_variable(
            var_type="_u", var_name="steering", shape=(1, 1)
        )
        # define states of the model
        self.x: SX | MX = self.model.set_variable(var_type="_x", var_name="x", shape=(1, 1))
        self.y: SX | MX = self.model.set_variable(var_type="_x", var_name="y", shape=(1, 1))
        self.psi: SX | MX = self.model.set_variable(var_type="_x", var_name="psi", shape=(1, 1))
        # constant parameter for the vehicle
        self.lFront: float = lFront
        self.lRear: float = lRear
        # define the time variable parameter of the reference
        self.xRef: SX | MX = self.model.set_variable(var_type="_tvp", var_name="x_ref")
        self.yRef: SX | MX = self.model.set_variable(var_type="_tvp", var_name="y_ref")
        self.psiRef: SX | MX = self.model.set_variable(var_type="_tvp", var_name="psi_ref")
        # define the controller Object
        self.pathFlag: bool = False
        # self.waypoints: Path = []
        self.lengthWaypoints: int = 0
        self.currentReferenceIndex: int = 0
        self.mpc = do_mpc.controller.MPC(self.model)
        self.tvpTemp1: SXStruct | MXStruct = self.mpc.get_tvp_template()
        self.simulator = do_mpc.simulator.Simulator(self.model)

    def stateCallback(self, msg: Odometry) -> None:
        """Callback function for the state of the vehicle"""
        self.state = msg

    def pathCallback(self, path: Path) -> None:
        """Callback function for the path of the vehicle"""
        self.waypoints = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                euler_from_quaternion(
                    [
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w,
                    ]
                )[2],
            )
            for pose in path.poses
        ]
        self.lengthWaypoints = len(self.waypoints)
        self.pathFlag = True

    def modelConfig(self) -> None:
        """
        Model configuration for the vehicle kinematics
        The model is a simple bicycle model with the following states:
        x: x position of the vehicle
        y: y position of the vehicle
        psi: orientation of the vehicle
        The model has two inputs:
        velocity: velocity of the vehicle
        steering: steering angle of the vehicle
        """
        xNext = self.vel * np.cos(self.psi)
        yNext = self.vel * np.sin(self.psi)
        psiNext = (self.vel / (self.lr + self.lf)) * tan(self.steering)
        self.model.set_rhs("x", xNext)
        self.model.set_rhs("y", yNext)
        self.model.set_rhs("psi", psiNext)
        # setup the model
        self.model.setup()

    def mpcConfigure(self) -> None:
        """
        MPC configuration for the vehicle control
        The MPC is configured with the following parameters:
        n_horizon: prediction horizon
        t_step: time step
        n_robust: number of robustness
        store_full_solution: store the full solution of the MPC
        """
        setupMpc = {
            "n_horizon": 15,
            "t_step": 0.1,
            "n_robust": 1,
            "store_full_solution": True,
        }
        self.mpc.set_param(**setupMpc)

    def costFunctionConfigure(
        self, xWeight: float = 10.0, yWeight: float = 10.0, psiWeight: float = 10.0
    ) -> None:
        """
        configure the cost function of the MPC controller
        The cost function is configured with the following parameters:
        wx: weight for the x position
        wy: weight for the y position
        wpsi: weight for the orientation
        """
        mTerm: SX | MX = (
            (xWeight * (self.x - self.xRef) ** 2)
            + (yWeight * (self.y - self.yRef) ** 2)
            + (psiWeight * (self.psi - self.psiRef) ** 2)
        )
        lTerm: SX | MX = mTerm
        self.mpc.set_objective(mterm=mTerm, lterm=lTerm)
        # set the change of penalty of the rate of change
        self.mpc.set_rterm(velocity=1.0, steering=1.0)

    def constrainMPC(
        self,
        steeringUpperLimit: float,
        steeringLowerLimit: float,
        velocityUpperLimit: float,
        velocityLowerLimit: float,
    ) -> None:
        """
        set the constraints of the MPC controller
        constrain the MPC controller with the following parameters:
        steeringUpperLimit: upper limit of the steering angle
        steeringLowerLimit: lower limit of the steering angle
        velocityUpperLimit: upper limit of the velocity
        velocityLowerLimit: lower limit of the velocity
        """
        self.mpc.bounds["lower", "_u", "steering"] = steeringLowerLimit
        self.mpc.bounds["upper", "_u", "steering"] = steeringUpperLimit
        self.mpc.bounds["lower", "_u", "velocity"] = velocityLowerLimit
        self.mpc.bounds["upper", "_u", "velocity"] = velocityUpperLimit

    def tvpFun(self) -> SXStruct | MXStruct:
        """
        time-varying parameters function for the MPC controller
        """
        return self.tvpTemp1

    def mpcFinalSetup(self) -> None:
        """
        Final setup of the MPC controller
        set the time-varying parameters function
        setup the MPC controller
        """
        self.tvpTemp1["_tvp", :] = [0.0, 0.0, 0.0]
        self.mpc.set_tvp_fun(self.tvpFun)
        self.mpc.setup()

    def ecludian(self, x: float, y: float, xRef: float, yRef: float) -> float:
        """
        calculate the ecludian distance between two points
        x: x position of the vehicle
        y: y position of the vehicle
        xRef: x position of the reference point
        yRef: y position of the reference point
        """
        return math.sqrt((xRef - x) ** 2 + (yRef - y) ** 2)

    def mpcSimulator(self) -> None:
        """
        MPC simulator for the vehicle control
        """
        self.tvpTemp1 = self.simulator.get_tvp_template()
        self.simulator.set_tvp_fun(self.tvpFun)

        self.simulator.set_param(t_step=0.05)

        self.simulator.setup()

    def run(self) -> None:
        """
        The function runs the MPC controller for the vehicle control
        """
        self.modelConfig()
        self.mpcConfigure()
        self.costFunctionConfigure()
        self.constrainMPC(-0.5, 0.5, 10, 0)
        self.mpcFinalSetup()
        self.mpcSimulator()

        while not self.pathFlag:
            self.mpcNode.spin_once(self.mpcNode)

        self.tvpTemp1["_tvp", :] = [
            self.waypoints[self.currentReferenceIndex][0],
            self.waypoints[self.currentReferenceIndex][1],
            self.waypoints[self.currentReferenceIndex][2],
        ]

        while rclpy.ok():
            self.mpcNode.spin_once(self.mpcNode)
            xInitial = [
                self.state.pose.pose.position.x,
                self.state.pose.pose.position.y,
                euler_from_quaternion(
                    [
                        self.state.pose.pose.orientation.x,
                        self.state.pose.pose.orientation.y,
                        self.state.pose.pose.orientation.z,
                        self.state.pose.pose.orientation.w,
                    ]
                )[2],
            ]
            self.mpc.set_initial_guess()
            controlCmd = self.mpc.make_step(x0=xInitial)
            self.controlMsg.steering_angle = controlCmd[0][1]
            self.controlMsg.speed = controlCmd[0][0]
            self.controlPub.publish(self.controlMsg)
            if (
                self.ecludian(
                    xInitial[0],
                    xInitial[1],
                    self.waypoints[self.currentReferenceIndex][0],
                    self.waypoints[self.currentReferenceIndex][0],
                )
                < 0.5
            ):
                self.currentReferenceIndex += 1
                if self.currentReferenceIndex >= self.lengthWaypoints:
                    break
                self.tvpTemp1["_tvp", :] = [
                    self.waypoints[self.currentReferenceIndex][0],
                    self.waypoints[self.currentReferenceIndex][1],
                    self.waypoints[self.currentReferenceIndex][2],
                ]

        self.controlMsg.steering_angle = 0.0
        self.controlMsg.speed = 0.0
        self.controlPub.publish(self.controlMsg)
        self.mpcNode.destroy_node()
        rclpy.shutdown()
