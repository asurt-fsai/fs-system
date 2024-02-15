import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path

## /steer float32    /throttle float32

## /state Odometry (State for rear wheel)

class Car(Node):
    """This class implements a kinematic bicycle model

    Attributes
    ----------
    x : numpy array
        state vector [x y theta delta]
    u : numpy array
        input vector [v delta_dot]
    xDot : numpy array
        state derivative vector [x_dot y_dot theta_dot delta_dot]
    dt : float
        time step
    wheelbase_length : float
        wheelbase length of the car
    timer : rclpy.timer.Timer
        timer to update the state of the car
    markerPub : rclpy.publisher.Publisher
        publisher to publish the marker array
    time : rclpy.time.Time
        time stamp used to sync the transforms and markers
    subscription : rclpy.subscription.Subscription
        subscription to the cmd_vel topic
    
    """
    def __init__(self,xInitial,dt = 0.1,wheelbase_length=2.5):
        super().__init__('KinematicBicycle')
        self.get_logger().info('Kinematic Bicycle Model Initialized')

        self.x = xInitial
        self.u = np.array([0.0,0.0])
        self.xDot = np.array([0,0,0,0])
        self.waypoint = PoseStamped()   
        self.dt = dt
        self.wheelbase_length = wheelbase_length
        self.timer = self.create_timer(self.dt, self.updateXDot)
        self.time = self.get_clock().now().to_msg()
        self.markerPub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.statePub = self.create_publisher(Odometry, '/state', 10)
        self.throttle = 0.0
        self.lastpointPub = self.create_publisher(Marker, 'last_point', 10)
        self.steeringSubscription = self.create_subscription(
            Float32,
            '/steer',
            self.steering_callback,
            10)
        self.throttleSubscription = self.create_subscription(
            Float32,
            '/throttle',
            self.throttle_callback,
            10)
        self.pathSubscription = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
    
    def steering_callback(self, msg:Float32):
        """This function is the callback function for the steering topic
        It updates the steering angle based on the message from the topic

        Parameters
        ----------
        msg : geometry_msgs.msg.Twist
            message from the cmd_vel topic
        
        Returns
        -------
        None
        """
        if (abs(msg.data) > 35):
            self.get_logger().info("STEERING ANGLE OUTSIDE LIMITS!, Try again....")
            self.destroy_node()
        self.x[3] = msg.data*np.pi/180
        
    def throttle_callback(self, msg:Float32):
        """This function is the callback function for the steering topic
        It updates the steering angle based on the message from the topic

        Parameters
        ----------
        msg : geometry_msgs.msg.Twist
            message from the cmd_vel topic
        
        Returns
        -------
        None
        """
        if (abs(msg.data) > 1.0):
            self.get_logger().info("THROTTLE OUTSIDE LIMITS!, Try again....")
            self.destroy_node()
        a = msg.data*(1.25 + 0.2*self.u[0] - 0.01*(self.u[0]**2))   
        self.u[0] = self.u[0] + a*self.dt

        self.throttle = msg.data
        

    def euler(self):
        """This function updates the state vector x using Euler integration
        x = x + x_dot*dt

        Parameters
        ----------
        None

        Returns
        -------
        None
        """

        self.x = self.x + self.xDot*self.dt
        
        
    def updateXDot(self):
        """This function updates the state derivative vector xDot

        x_dot = v*cos(theta)
        y_dot = v*sin(theta)
        theta_dot = v*tan(delta)/wheelbase_length
        delta_dot = delta_dot
        
        Parameters
        ----------
        None

        Returns
        -------
        None
        """        

        x_dot = self.u[0]*np.cos(self.x[2])
        y_dot = self.u[0]*np.sin(self.x[2])
        theta_dot = self.u[0]*np.tan(self.x[3])/self.wheelbase_length
        delta_dot = self.u[1]
        self.xDot = np.array([x_dot,y_dot,theta_dot,delta_dot])
        self.euler()
        self.updateTF()
        self.publishMarker()
        self.publishState()
        log = "Steering Angle: " + str(self.x[3]*180/np.pi) + " Throttle: " + str(self.throttle)
        self.get_logger().info(log)


    def path_callback(self, msg:Path):
        self.waypoint = msg.poses[-1]
        threshhold = 5.0
        distance = np.sqrt((self.x[0] - msg.poses[-1].pose.position.x)**2 + (self.x[1] - msg.poses[-1].pose.position.y)**2)

        #last point marker
        lastPoint = Marker()
        lastPoint.header.frame_id = "map"
        lastPoint.header.stamp = self.time
        lastPoint.ns = "last_point"
        lastPoint.id = 3
        lastPoint.type = Marker.SPHERE
        lastPoint.action = Marker.ADD
        lastPoint.pose.position.x = self.waypoint.pose.position.x#63.5
        #   self.waypoint.pose.position.x   # 5.0#self.waypoint[0]
        lastPoint.pose.position.y = self.waypoint.pose.position.y   #4.23 #self.waypoint.pose.position.y #15.0#self.waypoint[1]
        lastPoint.pose.position.z = -0.3
        lastPoint.pose.orientation.x = 0.0
        lastPoint.pose.orientation.y = 0.0
        lastPoint.pose.orientation.z = 0.0
        lastPoint.pose.orientation.w = 1.0
        lastPoint.scale.x = 6.0
        lastPoint.scale.y = 6.0
        lastPoint.scale.z = 0.1
        lastPoint.color.a = 1.0
        lastPoint.color.r = 1.0
        lastPoint.color.g = 0.5
        lastPoint.color.b = 0.0
        self.lastpointPub.publish(lastPoint)

        if (distance <= threshhold):
            if (self.u[0] == 0):
                self.stopTime += self.dt
            else:
                self.stopTime = 0
            if (self.stopTime > 3):
                lastPoint.color.g = 1.0
                lastPoint.color.r = 0.0
                lastPoint.color.b = 0.0
                
                self.get_logger().info("STOPPED FOR 3 SECONDS, CONGRATS!!!")
                self.destroy_node()

    def updateTF(self):
        """This function updates the frame transforms
        which takes care of the position calculation of the car
        given the state of one frame

        Tree:
        map
        |
        rear_link -----
        |             |
        | (static)    | (static)
        |             |
        front_link    base_link


        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # create transform broadcasters
        rlBroadcaster = tf2_ros.TransformBroadcaster(self)
        flBroadcaster = tf2_ros.TransformBroadcaster(self)
        blBroadcaster = tf2_ros.TransformBroadcaster(self)
        # initialize transform messages
        rlTF = geometry_msgs.msg.TransformStamped()
        flTF = geometry_msgs.msg.TransformStamped()
        blTF = geometry_msgs.msg.TransformStamped()
        # set time stamp
        self.time = self.get_clock().now().to_msg()

        # set rear link transform messages
        rlTF.header.stamp = self.time # set time stamp to current time
        rlTF.header.frame_id = "map" # set frame id to map
        rlTF.child_frame_id = "rear_link" # set child frame id to rear_link
        rlTF.transform.translation.x = self.x[0]
        rlTF.transform.translation.y = self.x[1]
        rlTF.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.x[2])
        rlTF.transform.rotation.x = q[0]
        rlTF.transform.rotation.y = q[1]
        rlTF.transform.rotation.z = q[2]
        rlTF.transform.rotation.w = q[3]

        flTF.header.stamp = rlTF.header.stamp
        flTF.header.frame_id = "rear_link"
        flTF.child_frame_id = "front_link"
        flTF.transform.translation.x = self.wheelbase_length
        flTF.transform.translation.y = 0.0
        flTF.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.x[3])
        flTF.transform.rotation.x = q[0]
        flTF.transform.rotation.y = q[1]
        flTF.transform.rotation.z = q[2]
        flTF.transform.rotation.w = q[3]

        blTF.header.stamp = rlTF.header.stamp
        blTF.header.frame_id = "rear_link"
        blTF.child_frame_id = "base_link"
        blTF.transform.translation.x = self.wheelbase_length/2
        blTF.transform.translation.y = 0.0
        blTF.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        blTF.transform.rotation.x = q[0]
        blTF.transform.rotation.y = q[1]
        blTF.transform.rotation.z = q[2]
        blTF.transform.rotation.w = q[3]

        globalBroadcaster = tf2_ros.TransformBroadcaster(self)

        # initialize transform messages
        globalTF = geometry_msgs.msg.TransformStamped()
        # set time stamp
        time = self.time
        # set rear link transform messages
        globalTF.header.stamp = time # set time stamp to current time
        globalTF.header.frame_id = "rear_link" # set frame id to map
        globalTF.child_frame_id = "path" # set child frame id to rear_link
        globalTF.transform.translation.x = -self.x[0]
        globalTF.transform.translation.y = -self.x[1]
        globalTF.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, -self.x[2])
        globalTF.transform.rotation.x = q[0]
        globalTF.transform.rotation.y = q[1]
        globalTF.transform.rotation.z = q[2]
        globalTF.transform.rotation.w = q[3]
        
        rlBroadcaster.sendTransform(rlTF)
        flBroadcaster.sendTransform(flTF)
        blBroadcaster.sendTransform(blTF)
        globalBroadcaster.sendTransform(globalTF)

    def publishMarker(self):
        """This function publishes the marker array

        Parameters
        ----------
        None

        Returns
        -------
        None
        """

        markerArray = MarkerArray() # create marker array

        # create marker for rear wheel
        rearWheel = Marker()
        rearWheel.header.frame_id = "rear_link" # set frame id to rear_link
        rearWheel.header.stamp =self.time # set time stamp to current time
        rearWheel.ns = "rear_wheel" # set namespace to rear_wheel
        rearWheel.id = 0 # set id to 0
        rearWheel.type = Marker.CUBE # set type to cube
        rearWheel.action = Marker.ADD # set action to add
        # set pose of marker
        rearWheel.pose.position.x = 0.0
        rearWheel.pose.position.y = 0.0
        rearWheel.pose.position.z = 0.0
        rearWheel.pose.orientation.x = 0.0
        rearWheel.pose.orientation.y = 0.0
        rearWheel.pose.orientation.z = 0.0
        rearWheel.pose.orientation.w = 1.0
        # set scale of marker
        rearWheel.scale.x = 1.0
        rearWheel.scale.y = 0.2
        rearWheel.scale.z = 1.0
        # set color of marker
        rearWheel.color.a = 1.0
        rearWheel.color.r = 1.0
        rearWheel.color.g = 0.0
        rearWheel.color.b = 0.0
        markerArray.markers.append(rearWheel) # append marker to marker array

        # create marker for front wheel
        frontWheel = Marker()
        frontWheel.header.frame_id = "front_link" # set frame id to front_link
        frontWheel.header.stamp = self.time # set time stamp to current time
        frontWheel.ns = "front_wheel" # set namespace to front_wheel
        frontWheel.id = 1 # set id to 1
        frontWheel.type = Marker.CUBE # set type to cube
        frontWheel.action = Marker.ADD # set action to add
        # set pose of marker
        frontWheel.pose.position.x = 0.0
        frontWheel.pose.position.y = 0.0
        frontWheel.pose.position.z = 0.0
        frontWheel.pose.orientation.x = 0.0
        frontWheel.pose.orientation.y = 0.0
        frontWheel.pose.orientation.z = 0.0
        frontWheel.pose.orientation.w = 1.0
        # set scale of marker
        frontWheel.scale.x = 1.0
        frontWheel.scale.y = 0.2
        frontWheel.scale.z = 1.0
        # set color of marker
        frontWheel.color.a = 1.0
        frontWheel.color.r = 1.0
        frontWheel.color.g = 0.0
        frontWheel.color.b = 0.0
        markerArray.markers.append(frontWheel) # append marker to marker array

        # create marker for car body
        carBody = Marker()
        carBody.header.frame_id = "bapublisherse_link" # set frame id to base_link
        carBody.header.stamp = self.time # set time stamp to current time
        carBody.ns = "car_body" # set namespace to car_body
        carBody.id = 2 # set id to 2
        carBody.type = Marker.CUBE # set type to cube
        carBody.action = Marker.ADD # set action to add
        # set pose of marker
        carBody.pose.position.x = 0.0
        carBody.pose.position.y = 0.0
        carBody.pose.position.z = 0.0
        carBody.pose.orientation.x = 0.0
        carBody.pose.orientation.y = 0.0
        carBody.pose.orientation.z = 0.0
        carBody.pose.orientation.w = 1.0
        # set scale of marker
        carBody.scale.x = self.wheelbase_length
        carBody.scale.y = 0.1
        carBody.scale.z = 0.1
        # set color of marker
        carBody.color.a = 1.0
        carBody.color.r = 0.0
        carBody.color.g = 0.0
        carBody.color.b = 1.0
        markerArray.markers.append(carBody) # append marker to marker array


        # publish marker array
        self.markerPub.publish(markerArray)

    def publishState(self):
        """This function publishes the state of the car

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        state = Odometry()
        state.header.stamp = self.time
        state.header.frame_id = "map"
        state.child_frame_id = "rear_link"
        state.pose.pose.position.x = self.x[0]
        state.pose.pose.position.y = self.x[1]
        state.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.x[2])
        state.pose.pose.orientation.x = q[0]
        state.pose.pose.orientation.y = q[1]
        state.pose.pose.orientation.z = q[2]
        state.pose.pose.orientation.w = q[3]
        state.twist.twist.linear.x = self.u[0]
        state.twist.twist.linear.y = 0.0
        state.twist.twist.linear.z = 0.0
        state.twist.twist.angular.x = 0.0
        state.twist.twist.angular.y = 0.0
        state.twist.twist.angular.z = 0.0
        self.statePub.publish(state)

