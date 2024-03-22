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



class LinearkinamaticMPC (Node):
    def __init__(self, l_f,l_r,n_horizon,t_step):
        ##Define nodes and Publisher and subscriber
        #self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        #self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.control_pub = self.create_publisher(AckermannDrive , "/control", 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_control_signals)
  
       #initalize Controller Variable
        self.n_horizon=n_horizon
        self.t_step=t_step
        #Initate the model and it's configurations
        model_type = 'continuous' 
        self.model = do_mpc.model.Model(model_type)
        #define the input of the model
        self.vel = self.model.set_variable(var_type='_u', var_name='velocity',shape=(1,1))
        self.steering = self.model.set_variable(var_type='_u', var_name='steering',shape=(1,1))
        #define states of the model
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        self.y = self.model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        self.psi = self.model.set_variable(var_type='_x', var_name='psi', shape=(1,1))
        self.beta = self.model.set_variable(var_type='_x', var_name='Beta', shape=(1,1))
        #constant parameter for the vehicle
        self.l_f = l_f
        self.l_r = l_r
        # define the time variable parameter of the reference
        self.x_ref = self.model.set_variable(var_type='_tvp', var_name='x_ref')
        self.y_ref = self.model.set_variable(var_type='_tvp', var_name='y_ref')
        self.psi_ref = self.model.set_variable(var_type='_tvp', var_name='psi_ref') 
        # define the controller Object
        
    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        velocity = math.sqrt(Vx ** 2 + Vy ** 2)
        X = state.pose.pose.position.x
        Y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.state = (X, Y, yaw, velocity)
        throttle_msg = Float32()
        steer_msg = Float32()  # Define steer_msg variable
        throttle_msg.data, steer_msg.data = self.mpc_control()
        self.control_pub.publish()

    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True

    def model_config(self):
        x_next = self.vel*np.cos(self.psi+self.beta)
        y_next = self.vel*np.sin(self.psi+self.beta)
        psi_next = self.vel/self.l_r*np.sin(self.beta)
        beta = np.arctan2(self.l_r/(self.l_f+self.l_r)*tan(self.steering))
        self.model.set_rhs('x',x_next)
        self.model.set_rhs('y',y_next)
        self.model.set_rhs('psi',psi_next)
        self.model.set_rhs('Beta',beta)
        #setup the model
        self.model.setup()
        pass

    def MPC_configure(self):
        self.mpc = do_mpc.controller.MPC(self.model)
        setup_mpc = {
            'n_horizon': 15,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)
    
    def costFunctionConfigure(self,wx=10,wy=10,wpsi=10):
        m_term=(wx(self.x-self.x_ref)**2)+(wy(self.y-self.y_ref)**2)+(wpsi(self.psi-self.psi_ref)**2)
        l_term=m_term
        self.mpc.set_objective(mterm=m_term, lterm=l_term)
        #set the change of penalty of the rate of change
        self.mpc.set_rterm(
            velocity=1.0,
            steering=1.0
        )

    def constrain(self,steering_upper_limit,steering_lower_limit,velocity_upper_limit,velocity_lower_limit):
        self.mpc.bounds['lower','_u','steering']=steering_lower_limit
        self.mpc.bounds['upper','_u','steering']=steering_upper_limit
        self.mpc.bounds['lower','_u','velocity']=velocity_lower_limit
        self.mpc.bounds['upper','_u','velocity']=velocity_upper_limit

    def MPC_Final_setup(self):
        self.tvp_temp_1 = self.mpc.get_tvp_template()
        self.tvp_temp_1['_tvp', :] = [0.0,0.0,0.0]
        self.mpc.set_tvp_fun(self.tvp_fun)
        self.mpc.setup()

    def tvp_fun(self,t_now):
        return self.tvp_temp_1
    

        
   

    
    
    