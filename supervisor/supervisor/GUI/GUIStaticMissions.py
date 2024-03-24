from tkinter import *
import tkinter as tk
from threading import Thread
from PIL import ImageTk, Image
import rclpy
from std_msgs.msg import Float32, Bool, Int16

root = Tk()

root.title("Formula 1 AI")
root.geometry("1400x600") 
root.configure(bg = 'black')

color1 = '#020f12'
color2 = '#8b0000'
color3 = '#a60000'
color4 = 'black'

img1 = Image.open('fs-system/supervisor/supervisor/GUI/formulaIcon.png').resize((200,100), Image.ANTIALIAS)
img1_tk = ImageTk.PhotoImage(img1)
imgLabel1 = Label(root, image=img1_tk, background='black')
imgLabel1.image = img1_tk 
imgLabel1.place(relx=.15, rely=0.1, anchor=CENTER)

img2 = Image.open('fs-system/supervisor/supervisor/GUI/racingTeamIcon.png').resize((200,100), Image.ANTIALIAS)
img2_tk = ImageTk.PhotoImage(img2)
imgstateLabel = Label(root, image=img2_tk, background='black')
imgstateLabel.image = img2_tk  
imgstateLabel.place(relx=.85, rely=0.1, anchor=CENTER)

def ros_spinner():
    rclpy.init()
    node = rclpy.create_node('tkinter_node')
    velSub = node.create_subscription(Float32, '/control/velocity', velCallback, 10)
    steerSub = node.create_subscription(Float32, '/control/steering', steerCallback, 10)
    stateSub = node.create_subscription(Int16, '/state', stateCallback, 10)

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
        ("Mission Type", .2, .40),
        ("AS state", .4, .40),
        ("Velocity", .6, .40),
        ("Steering", .8, .40),
    ]

for text, relx, rely in labels:
    label = Label(root, text=text, font=("aptos", 12, 'bold'),background='black', fg = 'white' )
    label.place(relx=relx, rely=rely, anchor=CENTER)


missionlabel = tk.Label(root, text="Static A", font=("aptos", 11), background='black', fg='white')
missionlabel.place(relx=.2, rely=.6, anchor=tk.CENTER)

stateLabel = tk.Label(root, text="state", font=("aptos", 11), background='black', fg='white')
stateLabel.place(relx=.4, rely=.6, anchor=tk.CENTER)

velcityLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
velcityLabel.place(relx=.6, rely=.6, anchor=tk.CENTER)

steeringLabel = tk.Label(root, text="0", font=("aptos", 11), background='black', fg='white')
steeringLabel.place(relx=.8, rely=.6, anchor=tk.CENTER)

# label = Label(root , text="velocity", font=("aptos", 12, 'bold'),background='black', fg = 'white' )
# label.place(relx=.26, rely=.33, anchor=CENTER)
# velcityLabel = tk.Label(root, text= "hello", font=("aptos", 11), background='black', fg='white')
# velcityLabel.place(relx=.26, rely=.70, anchor=tk.CENTER)

def velCallback(msg: Float32):
    message = str("%.2f" % msg.data )
    print(message)
    velcityLabel.config(text=message)

def steerCallback(msg: Float32):
    message = str("%.2f" % msg.data )
    steeringLabel.config(text=message)

def stateCallback(msg: Int16):
    print(msg)
    message = str( msg.data )
    if message == '1':
        stateLabel.config(text = "driving")    
    elif message == '2':
         stateLabel.config(text = "finished")    
    
     


start_ros_spinner()
root.mainloop()