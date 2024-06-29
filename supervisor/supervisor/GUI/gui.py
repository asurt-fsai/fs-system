from tkinter import *
import tkinter as tk
from threading import Thread
from PIL import ImageTk, Image
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
from asurt_msgs.msg import NodeStatus
from supervisor.helpers.module import Module

# Initialize ROS 2
rclpy.init()

root = Tk()
root.title("Formula 1 AI")
root.geometry("1800x1200")
root.configure(bg='black')

color1 = '#020f12'
color2 = '#8b0000'
color3 = '#a60000'
color4 = 'black'

img1 = Image.open('/home/yomnahashem/Formula24/src/fs-system/supervisor/supervisor/GUI/formulaIcon.png').resize((200, 100), Image.ANTIALIAS)
img1_tk = ImageTk.PhotoImage(img1)
imgLabel1 = Label(root, image=img1_tk, background='black')
imgLabel1.place(relx=.15, rely=0.1, anchor=CENTER)

img2 = Image.open('/home/yomnahashem/Formula24/src/fs-system/supervisor/supervisor/GUI/racingTeamIcon.png').resize((200, 100), Image.ANTIALIAS)
img2_tk = ImageTk.PhotoImage(img2)
imgstateLabel = Label(root, image=img2_tk, background='black')
imgstateLabel.place(relx=.85, rely=0.1, anchor=CENTER)

# Node to handle subscriptions
class ROSNode(Node):
    def __init__(self):
        super().__init__('tkinter_node')
        self.create_subscription(Float32, '/control/velocity', self.vel_callback, 10)
        self.create_subscription(Float32, '/control/steering', self.steer_callback, 10)
        self.create_subscription(Int16, '/state', self.state_callback, 10)
        self.create_subscription(Float32, '/distance', self.distance_callback, 10)

    def vel_callback(self, msg: Float32):
        velocityLabel.config(text=f"{msg.data:.2f}")

    def steer_callback(self, msg: Float32):
        steeringLabel.config(text=f"{msg.data:.2f}")

    def distance_callback(self, msg: Float32):
        distanceLabel.config(text=f"{msg.data:.2f}")

    def state_callback(self, msg: Int16):
        state = {2: "driving", 3: "EBS", 4: "finished"}.get(msg.data, "unknown")
        ASstateLabel.config(text=state)

def ros_spinner():
    node = ROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def start_ros_spinner():
    ros_thread = Thread(target=ros_spinner)
    ros_thread.start()

labels = [
    ("Mission Type", .15, .20),
    ("AS state", .37, .20),
    ("SLAM loop closure", .62, .20),
    ("Lap Count", .85, .20),
    ("Velocity", .15, .35),
    ("Steering", .37, .35),
    ("Distance", .62, .35),
    ("Acceleration", .85, .35)
]

for text, relx, rely in labels:
    label = Label(root, text=text, font=("aptos", 12, 'bold'), background='black', fg='white')
    label.place(relx=relx, rely=rely, anchor=CENTER)

missionlabel = tk.Label(root, text="Static A", font=("aptos", 11), background='black', fg='white')
missionlabel.place(relx=.15, rely=.25, anchor=tk.CENTER)

ASstateLabel = tk.Label(root, text="State", font=("aptos", 11), background='black', fg='white')
ASstateLabel.place(relx=.37, rely=.25, anchor=tk.CENTER)

loopLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
loopLabel.place(relx=.62, rely=.25, anchor=tk.CENTER)

lapCountLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
lapCountLabel.place(relx=.85, rely=.25, anchor=tk.CENTER)

velocityLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
velocityLabel.place(relx=.15, rely=.4, anchor=tk.CENTER)

steeringLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
steeringLabel.place(relx=.37, rely=.4, anchor=tk.CENTER)

distanceLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
distanceLabel.place(relx=.62, rely=.4, anchor=tk.CENTER)

accelerationLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
accelerationLabel.place(relx=.85, rely=.4, anchor=tk.CENTER)

LaunchedModulesLabel = tk.Label(root, text="Launched Modules", font=("aptos", 12, 'bold'), background='black', fg='#820404')
LaunchedModulesLabel.place(relx=.25, rely=.5, anchor=tk.CENTER)

stateLabel = tk.Label(root, text="State", font=("aptos", 12, 'bold'), background='black', fg='#820404')
stateLabel.place(relx=.5, rely=.5, anchor=tk.CENTER)

freqLabel = tk.Label(root, text="Frequency", font=("aptos", 12, 'bold'), background='black', fg='#820404')
freqLabel.place(relx=.75, rely=.5, anchor=tk.CENTER)

states = ["starting", "ready", "running", "error", "shutdown", "unresponsive"]

modules = [
    Module(pkg="example_pkg1", launchFile="example_launch_file1"),
    Module(pkg="example_pkg2", launchFile="example_launch_file2"),
    Module(pkg="example_pkg3", launchFile="example_launch_file3"),
]

def getTextColor(nodeStatus: int) -> str:
    if nodeStatus in [NodeStatus.STARTING, NodeStatus.READY]:
        return "yellow"
    if nodeStatus in [NodeStatus.ERROR, NodeStatus.SHUTDOWN, NodeStatus.UNRESPONSIVE]:
        return "white"
    return "white"

for index, module in enumerate(modules):
    color = getTextColor(module.state)

    Modulelabel = tk.Label(root, text=module.pkg, font=("aptos", 12), bg='black', fg=color)
    Modulelabel.place(relx=0.25, rely=0.6 + 0.07 * index, anchor=tk.CENTER)

    Modulelabel = tk.Label(root, text=states[module.state], font=("aptos", 12), bg='black', fg=color)
    Modulelabel.place(relx=0.5, rely=0.6 + 0.07 * index, anchor=tk.CENTER)

    Modulelabel = tk.Label(root, text=f"{module.rate:.2f} hz", font=("aptos", 12), bg='black', fg=color)
    Modulelabel.place(relx=0.75, rely=0.6 + 0.07 * index, anchor=tk.CENTER)

start_ros_spinner()
root.mainloop()








# from tkinter import *
# import tkinter as tk
# from threading import Thread
# from PIL import ImageTk, Image
# import rclpy
# from std_msgs.msg import Float32, Bool, Int16
# from supervisor.helpers.module import Module
# from asurt_msgs.msg import NodeStatus

# root = Tk()


# root.title("Formula 1 AI")
# root.geometry("1800x1200") 
# root.configure(bg = 'black')

# color1 = '#020f12'
# color2 = '#8b0000'
# color3 = '#a60000'
# color4 = 'black'

# img1 = Image.open('/home/yomnahashem/Formula24/src/fs-system/supervisor/supervisor/GUI/formulaIcon.png').resize((200,100), Image.ANTIALIAS)
# img1_tk = ImageTk.PhotoImage(img1)
# imgLabel1 = Label(root, image=img1_tk, background='black') 
# imgLabel1.place(relx=.15, rely=0.1, anchor=CENTER)

# img2 = Image.open('/home/yomnahashem/Formula24/src/fs-system/supervisor/supervisor/GUI/racingTeamIcon.png').resize((200,100), Image.ANTIALIAS)
# img2_tk = ImageTk.PhotoImage(img2)
# imgstateLabel = Label(root, image=img2_tk, background='black')
# imgstateLabel.place(relx=.85, rely=0.1, anchor=CENTER)

# def ros_spinner():
#     rclpy.init()
#     node = rclpy.create_node('tkinter_node')
#     node.create_subscription(Float32, '/control/velocity', velCallback, 10)
#     node.create_subscription(Float32, '/control/steering', steerCallback, 10)
#     node.create_subscription(Int16, '/state', stateCallback, 10)
#     node.create_subscription(Float32, '/distance', distanceCallback, 10)                      

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# def start_ros_spinner():
#     ros_thread = Thread(target=ros_spinner)
#     ros_thread.start()



# labels = [
#         ("Mission Type", .15, .20),
#         ("AS state", .37, .20),
#         ("SLAM loop closure", .62, .20),
#         ("Lap Count", .85, .20),
#         ("Velocity", .15, .35),
#         ("Steering", .37, .35),
#         ("Distance", .62, .35),
#         ("Acceleration",.85, .35)
#     ]

# for text, relx, rely in labels:
#     label = Label(root, text=text, font=("aptos", 12, 'bold'),background='black', fg = 'white' )
#     label.place(relx=relx, rely=rely, anchor=CENTER)

    
# missionlabel = tk.Label(root, text="Static A", font=("aptos", 11), background='black', fg='white')
# missionlabel.place(relx=.15, rely=.25, anchor=tk.CENTER)

# ASstateLabel = tk.Label(root, text="State", font=("aptos", 11), background='black', fg='white')
# ASstateLabel.place(relx=.37, rely=.25, anchor=tk.CENTER)

# loopLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
# loopLabel.place(relx=.62, rely=.25, anchor=tk.CENTER)

# lapCountLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
# lapCountLabel.place(relx=.85, rely=.25, anchor=tk.CENTER)

# velocityLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
# velocityLabel.place(relx=.15, rely=.4, anchor=tk.CENTER)

# steeringLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
# steeringLabel.place(relx=.37, rely=.4, anchor=tk.CENTER)

# distanceLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
# distanceLabel.place(relx=.62, rely=.4, anchor=tk.CENTER)

# accelerationLabel = tk.Label(root, text="N/A", font=("aptos", 11), background='black', fg='white')
# accelerationLabel.place(relx=.85, rely=.4, anchor=tk.CENTER)

# LaunchedModulesLabel = tk.Label(root, text="Launched Modules", font=("aptos", 12, 'bold'), background='black', fg='#820404')
# LaunchedModulesLabel.place(relx=.25, rely=.5, anchor=tk.CENTER)

# stateLabel = tk.Label(root, text="State", font=("aptos", 12, 'bold'), background='black', fg='#820404')
# stateLabel.place(relx=.5, rely=.5, anchor=tk.CENTER)

# freqLabel = tk.Label(root, text="Frequency", font=("aptos", 12, 'bold'), background='black', fg='#820404')
# freqLabel.place(relx=.75, rely=.5, anchor=tk.CENTER)



# states = ["starting", "ready", "running", "error", "shutdown", "unreponsive"]

# modules = [
#     Module(pkg="example_pkg1", launchFile="example_launch_file1"),
#     Module(pkg="example_pkg2", launchFile="example_launch_file2"),
#     Module(pkg="example_pkg3", launchFile="example_launch_file3"),
# ]

# def getTextColor(nodeStatus: int) -> str:
#     if nodeStatus in [NodeStatus.STARTING, NodeStatus.READY]:
#         return "yellow"
#     if nodeStatus in [NodeStatus.ERROR, NodeStatus.SHUTDOWN, NodeStatus.UNRESPONSIVE]:
#         return "white"
#     return "white"

# for index, module in enumerate(modules):
#     color = getTextColor(module.state)

#     Modulelabel = tk.Label(root, text = module.pkg, font=("aptos", 12), bg='black', fg = color )
#     Modulelabel.place(relx= 0.25, rely= 0.6 + 0.07*index, anchor= tk.CENTER)

#     Modulelabel = tk.Label(root, text = states[module.state], font=("aptos", 12), bg='black', fg = color )
#     Modulelabel.place(relx= 0.5, rely= 0.6 + 0.07*index, anchor= tk.CENTER)

#     Modulelabel = tk.Label(root, text = f"{module.rate:.2f} hz", font=("aptos", 12), bg='black', fg = color )
#     Modulelabel.place(relx= 0.75, rely= 0.6 + 0.07*index, anchor= tk.CENTER)



# def velCallback(msg: Float32):
#     message = str("%.2f" % msg.data )
#     velocityLabel.config(text=message)

# def steerCallback(msg: Float32):
#     message = str("%.2f" % msg.data )
#     steeringLabel.config(text=message)

# def distanceCallback(msg: Float32):
#     message = str("%.2f" % msg.data )
#     distanceLabel.config(text=message)

# def stateCallback(msg: Int16):
#     message = str( msg.data )
#     if message == '2':
#         ASstateLabel.config(text = "driving")
#     elif message == '3':
#         ASstateLabel.config(text = "EBS")  
#     elif message == '4':
#          ASstateLabel.config(text = "finished")    
        

# start_ros_spinner()
# root.mainloop()