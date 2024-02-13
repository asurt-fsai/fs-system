import do_mpc
import numpy as np
import math
import rclpy
from casadi import *
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from tf_transformations import euler_from_quaternion

#dynamic model
#tazbet elconstrains
#meen input w meen output w hygolna mnen
#eh elhagat ely da5la 3al mpc w by5arag eh w no3 eldata ely 5arga eh
#output (Target Speed, Steering)

class control_mpc(Node):
    def _init_(self):
        super().__init__('control_mpc')

        #self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        #self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.control_pub = self.create_publisher(AckermannDrive , "/control", 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_control_signals)
        
        #hn3araf ay haga feha self , x_pos w kda
        
    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        self.velocity = math.sqrt(Vx ** 2 + Vy ** 2)
        X = state.pose.pose.position.x
        Y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.state = (X, Y, yaw)
        throttle_msg = Float32()
        throttle_msg.data, steer_msg.data = self.mpc_control()
        self.control_pub.publish()

    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True

    def mpc_control (self,vehicle):
        self.vehicle = vehicle           ######
        self.model = vehicle.model     #l7d ma ashof hwarat el model

        self.horizon = 15
        ##globals.horizon = self.horizon  

        self.Ts = 0.1
        
        self.current_prediction = None

        self.mpc = do_mpc.controller.MPC(self.model)
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': self.horizon,
            't_step': self.Ts,
        }
        self.mpc.set_param(**setup_mpc)

        # define the objective function and constriants
        self.objective_function_setup()
        self.constraints_setup()

        # provide time-varing parameters: setpoints/references
        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.setup()
    def constraints_setup(self, reset=False):

        # states constraints
        self.mpc.bounds['lower', '_x', 'pos_x'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_x'] = np.inf
        self.mpc.bounds['lower', '_x', 'pos_y'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_y'] = np.inf
        # self.mpc.bounds['lower', '_x', 'steer'] = - 30
        # self.mpc.bounds['upper', '_x', 'steer'] = 30
        self.mpc.bounds['lower', '_x', 'vel']   = -0.5            
        self.mpc.bounds['upper', '_x', 'vel']   = 0.5
       

        # input constraints
        self.mpc.bounds['lower', '_u', 'acc'] = -0.5
        self.mpc.bounds['upper', '_u', 'acc'] = 0.5
        self.mpc.bounds['lower', '_u', 'delta'] = -30
        self.mpc.bounds['upper', '_u', 'delta'] = 30

        if reset is True:
            self.mpc.setup()    

    def objective_function_setup(self):
        lterm = (self.model.aux[self.psi_diff] ** 2
                + (self.model.x[self.pos_x] - self.model.tvp[self.x_ref]) ** 2
                + (self.model.x[self.pos_y] - self.model.tvp[self.y_ref]) ** 2
                +  0.1 * (self.model.x[self.vel] - self.model.tvp[self.vel_ref]) ** 2)
            
        mterm = ((self.model.x[self.pos_x] - self.model.tvp[self.x_ref]) ** 2
                + (self.model.x[self.pos_y] - self.model.tvp[self.y_ref]) ** 2
                +  0.1 * (self.model.x[self.vel] - self.model.tvp[self.vel_ref]) ** 2)
        self.mpc.set_objective(mterm=mterm, lterm=lterm)

    def model_setup(self,path ,Ts):
        model_type = "discrete"  # either 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)

        # States struct (optimization variables):
        self.pos_x = self.model.set_variable(var_type="_x", var_name="pos_x")
        self.pos_y = self.model.set_variable(var_type="_x", var_name="pos_y")
        steer = self.model.set_variable(var_type="_x", var_name="steer")
        vel = self.model.set_variable(var_type="_x", var_name="vel")
        

        # Input struct (optimization variables):
        acc = self.model.set_variable(var_type="_u", var_name="acc")
        delta = self.model.set_variable(var_type="_u", var_name="delta")

        # reference data
        # using time-varing parameters data type
        self.x_ref = self.model.set_variable(var_type="_tvp", var_name="x_ref")
        self.y_ref = self.model.set_variable(var_type="_tvp", var_name="y_ref")
        self.psi_ref = self.model.set_variable(var_type="_tvp", var_name="psi_ref")
        self.vel_ref = self.model.set_variable(var_type="_tvp", var_name="vel_ref")

        # tracking errors (optimization variables):
        psi_diff = (fmod(steer - self.psi_ref + np.pi, 2 * np.pi) - np.pi)
        self.model.set_expression('psi_diff', psi_diff)

        self.model.set_rhs("pos_x", vel * cos(steer))
        self.model.set_rhs("pos_y", vel * sin(steer))
        self.model.set_rhs("steer", vel * delta / self.length)
        self.model.set_rhs("vel", acc)

        self.model.setup()
    
    
    