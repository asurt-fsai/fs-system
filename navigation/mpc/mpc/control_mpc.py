import do_mpc
import numpy as np
import math
import rclpy
from casadi import *
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from tf_transformations import euler_from_quaternion



class LinearkinamaticMPC (Node):
    def __init__(self, l_f,l_r,n_horizon,t_step):
        ##Define nodes and Publisher and subscriber
        rclpy.init(args=None)
        self.MPC_NODE = rclpy.create_node('MPC_NODE')
        self.control_pub = self.MPC_NODE.create_publisher(AckermannDrive , "/control", 10)
        self.state_sub = self.MPC_NODE.create_subscription(Odometry, '/state', self.state_callback, 10)
        self.path_sub = self.MPC_NODE.create_subscription(Path, '/path', self.path_callback, 10)
        self.waypoints = Path() #initialize the waypoints 
        self.state = Odometry() #initialize the state
        self.control_msg = AckermannDrive() #initialize the control
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
        #constant parameter for the vehicle
        self.l_f = l_f
        self.l_r = l_r
        # define the time variable parameter of the reference
        self.x_ref = self.model.set_variable(var_type='_tvp', var_name='x_ref')
        self.y_ref = self.model.set_variable(var_type='_tvp', var_name='y_ref')
        self.psi_ref = self.model.set_variable(var_type='_tvp', var_name='psi_ref') 
        # define the controller Object
        self.pathFlag=False
        self.waypoints=[]
        self.length_of_waypoints=[]
        self.current_reference_index=0
        
    def state_callback(self, msg: Odometry):
        self.state = msg

    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x,
                           pose.pose.position.y,
                           euler_from_quaternion([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])[2]) for pose in path.poses]
        self.length_of_waypoints = len(self.waypoints)
        self.pathFlag = True

    def model_config(self):
        x_next = self.vel*np.cos(self.psi)
        y_next = self.vel*np.sin(self.psi)
        psi_next = (self.vel/(self.l_r+self.l_f))*tan(self.steering)
        self.model.set_rhs('x',x_next)
        self.model.set_rhs('y',y_next)
        self.model.set_rhs('psi',psi_next)
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

    def constrainMPC(self,steering_upper_limit,steering_lower_limit,velocity_upper_limit,velocity_lower_limit):
        self.mpc.bounds['lower','_u','steering']=steering_lower_limit
        self.mpc.bounds['upper','_u','steering']=steering_upper_limit
        self.mpc.bounds['lower','_u','velocity']=velocity_lower_limit
        self.mpc.bounds['upper','_u','velocity']=velocity_upper_limit

    def tvp_fun(self,t_now):
        return self.tvp_temp_1
    
    def MPC_Final_setup(self):
        self.tvp_temp_1 = self.mpc.get_tvp_template()
        self.tvp_temp_1['_tvp', :] = [0.0,0.0,0.0]
        self.mpc.set_tvp_fun(self.tvp_fun)
        self.mpc.setup()

    def ecludian (self , x ,y ,x_ref,y_ref):
        return math.sqrt((self.x_ref-self.x)**2+(self.y_ref-self.y)**2) 
    def simulator_MPC(self):
        self.simulator = do_mpc.simulator.Simulator(self.model)

        # provide time-varing parameters: setpoints/references
        self.tvp_temp_1 = self.simulator.get_tvp_template()
        self.simulator.set_tvp_fun(self.tvp_fun)

        self.simulator.set_param(t_step=0.05)

        self.simulator.setup()

    
    def run(self):
        self.model_config()
        self.MPC_configure()
        self.costFunctionConfigure()
        self.constrainMPC(-0.5,0.5,10,0)
        self.MPC_Final_setup()
        self.simulator_MPC()

        while not self.pathFlag:
            self.MPC_NODE.spin_once(self.MPC_NODE)

        self.tvp_temp_1['_tvp', :] = [self.waypoints[self.current_reference_index][0],
                                      self.waypoints[self.current_reference_index][1],
                                      self.waypoints[self.current_reference_index][2]]
        
        while rclpy.ok():
            self.MPC_NODE.spin_once(self.MPC_NODE)
            x0=[self.state.pose.pose.position.x,
                         self.state.pose.pose.position.y,
                         euler_from_quaternion([self.state.pose.pose.orientation.x,self.state.pose.pose.orientation.y,self.state.pose.pose.orientation.z,self.state.pose.pose.orientation.w])[2]]
            self.mpc.set_initial_guess()
            control_cmd = self.mpc.make_step(x0=x0)
            self.control_msg.steering_angle = control_cmd[0][1]
            self.control_msg.speed = control_cmd[0][0] 
            self.control_pub.publish(self.control_msg)
            if self.ecludian(x0[0],x0[1],self.waypoints[self.current_reference_index][0],self.waypoints[self.current_reference_index][0])<0.5:
                self.current_reference_index+=1
                if self.current_reference_index>=self.length_of_waypoints:
                    break
                self.tvp_temp_1['_tvp', :] = [self.waypoints[self.current_reference_index][0],
                                      self.waypoints[self.current_reference_index][1],
                                      self.waypoints[self.current_reference_index][2]]
                
        self.control_msg.steering_angle = 0.0
        self.control_msg.speed = 0.0 
        self.control_pub.publish(self.control_msg)
        self.MPC_NODE.destroy_node()
        rclpy.shutdown()





        
 
    

        
   

    
    
    