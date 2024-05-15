import tkinter as tk
from tkinter import *
import rclpy
from std_msgs.msg import Bool, String
from PIL import ImageTk, Image
from eufs_msgs.msg import CanState

class Interface:
    def __init__(self, master):
        self.master = master
        master.title("IPG MODULE INTERFACE")
        master.geometry("1400x1000")
        self.init_ros()
        self.create_widgets()
        self.ASmaster = False
        self.TSmaster = False
        self.Flag = False
        self.drivingFlag = False
        self.GoSignal = False
        self.EBS = True
        self.EBSfail= True
        self.misson = 0
        self.state = 0
        self.sdc = Bool()
        self.sdc.data = True
        




    def init_ros(self):
        rclpy.init()

        self.node = rclpy.create_node("interface")

        self.as_master_pub = self.node.create_publisher(Bool, "/state_machine/as_master_switch", 1)
        self.ts_master_pub = self.node.create_publisher(Bool, "/state_machine/ts_master_switch", 1)
        self.go_signal_pub = self.node.create_publisher(Bool, "/state_machine/go_signal", 1)
        self.ebs_pub = self.node.create_publisher(Bool, "/state_machine/ebs", 1)
        self.ebs_fail_pub = self.node.create_publisher(Bool, "/state_machine/ebs_fail", 1)
        self.flag_pub = self.node.create_publisher(Bool, "/state_machine/flag", 1)
        self.driving_flag_pub = self.node.create_publisher(Bool, "/state_machine/driving_flag", 1)
        self.canStatePub = self.node.create_publisher(CanState,"/ros_can/state", 1)

        self.state_sub = self.node.create_subscription(String, "/state_machine/state_str", self.state_cb, 10)
        self.sdc_sub =self.node.create_subscription(Bool, "shutDownCircuit", self.sdcCB, 10)
        
        self.CanStateMsg = CanState()
        
    def sdcCB(self, msg: Bool):
        self.node.get_logger().info("1")
        self.sdc = msg
        self.node.get_logger().info("2")
        self.node.get_logger().info(str(self.sdc))  
        self.update()

    def state_cb(self, msg: String):
        self.text.config(text=msg.data)

    def create_widgets(self):


        self.color1 = '#020f12'
        self.color2 = '#8b0000'
        self.color3 = '#a60000'
        self.color4 = 'black'
        self.color5 = "#195e2f"


        self.text = tk.Label(
            self.master,
            text="", bg = self.color4,
            justify="center", foreground="white", font=("ubuntu", 18)
        )
        self.text.place(relx=0.5, rely=0.05, anchor="n")

        self.as_master_button = tk.Button(self.master, text="AS MASTER: OFF", width=24, height=3,font=("ubuntu", 12), bg = self.color2, borderwidth=0, highlightthickness=0)
        self.as_master_button.configure(command=self.toggle_as_master)
        self.as_master_button.place(relx=0.15, rely=0.15)

        self.ts_master_button = tk.Button(self.master, text="TS MASTER: OFF", width=24, height=3, font=("ubuntu", 12),bg = self.color2, borderwidth=0, highlightthickness=0)
        self.ts_master_button.configure(command=self.toggle_ts_master)
        self.ts_master_button.place(relx=0.55, rely=0.15)

        self.ebs_button = tk.Button(self.master, text="EBS: ARMED", width=12, height=3,font=("ubuntu", 10), bg=self.color5, borderwidth=0, highlightthickness=0)
        self.ebs_button.configure(command=self.toggle_ebs)
        self.ebs_button.place(relx=0.55, rely=0.3)

        self.ebs_fail_button = tk.Button(self.master, text="EBS: AVAILABLE",font=("ubuntu", 10), width=14, height=3, bg=self.color5, borderwidth=0, highlightthickness=0)
        self.ebs_fail_button.configure(command=self.toggle_ebs_fail)
        self.ebs_fail_button.place(relx=0.71, rely=0.3)

        self.flag_button = tk.Button(self.master, text="FLAG: OFF", width=24, height=3, font=("ubuntu", 12),bg = self.color2, borderwidth=0, highlightthickness=0)
        self.flag_button.configure(command=self.toggle_flag)
        self.flag_button.place(relx=0.15, rely=0.3)

        self.driving_flag_button = tk.Button(self.master, text="DRIVING FLAG: OFF", width=24, height=3, font=("ubuntu", 12),bg = self.color2, borderwidth=0, highlightthickness=0)
        self.driving_flag_button.configure(command=self.toggle_driving_flag)
        self.driving_flag_button.place(relx=0.15, rely=0.45)

        self.go_signal_button = tk.Button(self.master, text="GO SIGNAL: OFF", width=24, height=3, font=("ubuntu", 12), bg = self.color2, borderwidth=0, highlightthickness=0)
        self.go_signal_button.configure(command=self.toggle_go_signal)
        self.go_signal_button.place(relx=0.55, rely=0.45)

        buttons = [
            ("Not selected", 0.2, 0.7, self.notSelectedCommand),
            ("Static A", 0.4, 0.7, self.staticAcommand),
            ("Static B", 0.6, 0.7, self.staticBcommand),
            ("Autocross", 0.8, 0.7, self.autocrossCommand),
            ("autoDemo", 0.2, 0.85, self.autoDemoCommand),
            ("Skid Pad", 0.4, 0.85, self.skidpadCommand),
            ("Track Drive", 0.6, 0.85, self.trackDriveCommand),
            ("Acceleration", 0.8, 0.85, self.accelerationCommand)
        ]

        for text, relx, rely, command_func in buttons:
            button = Button(self.master, text=text, font=("ubuntu", 12), background=self.color1,
                            foreground=self.color2, activebackground=self.color3, activeforeground=self.color4,
                            highlightbackground=self.color2, highlightthickness=2, highlightcolor='white',
                            width=12, height=2, command=command_func)
            button.place(relx=relx, rely=rely, anchor=CENTER)

    def notSelectedCommand(self):
        self.text.configure(text="Not selected")
        self.CanStateMsg.ami_state = 10
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 10
   

    def staticAcommand(self):
        self.text.configure(text="Static A")
        self.CanStateMsg.ami_state = 18
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 18

    def staticBcommand(self):
        self.text.configure(text="Static B")
        self.CanStateMsg.ami_state = 19
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 19

    def autocrossCommand(self):
        self.text.configure(text="Autocross")
        self.CanStateMsg.ami_state = 13
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 13

    def autoDemoCommand(self):
        self.text.configure(text="Autonomus Demo")
        self.CanStateMsg.ami_state = 15
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 15

    def skidpadCommand(self):
        self.text.configure(text="Skidpad")
        self.CanStateMsg.ami_state = 12
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 12

    def trackDriveCommand(self):
        self.text.configure(text="Track Drive")
        self.CanStateMsg.ami_state = 14
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 14

    def accelerationCommand(self):
        self.text.configure(text="Acceleration")
        self.CanStateMsg.ami_state = 11
        self.canStatePub.publish(self.CanStateMsg)
        self.misson = 11

    def toggle_as_master(self):
        msg = Bool()
        msg.data = not self.as_master_button["text"].endswith("ON")
        self.as_master_pub.publish(msg)
        self.as_master_button.config(text="AS MASTER: ON" if msg.data else "AS MASTER: OFF", bg=self.color5 if msg.data else self.color2)
        self.ASmaster = not self.ASmaster
        #self.update(self.ASmaster, self.TSmaster, self.drivingFlag, self.Flag, self.EBS, self.EBSfail, self.GoSignal, self.misson)
        self.update()
    def toggle_ts_master(self):
        msg = Bool()
        msg.data = not self.ts_master_button["text"].endswith("ON")
        self.ts_master_pub.publish(msg)
        self.ts_master_button.config(text="TS MASTER: ON" if msg.data else "TS MASTER: OFF", bg=self.color5 if msg.data else self.color2)
        self.TSmaster = not self.TSmaster

    def toggle_go_signal(self):
        msg = Bool()
        msg.data = not self.go_signal_button["text"].endswith("ON")
        self.go_signal_pub.publish(msg)
        self.go_signal_button.config(text="GO SIGNAL: ON" if msg.data else "GO SIGNAL: OFF", bg=self.color5 if msg.data else self.color2)

    def toggle_ebs(self):
        msg = Bool()
        msg.data = not self.ebs_button["text"].endswith("TRIGGERED")
        self.ebs_pub.publish(msg)
        self.ebs_button.config(text="EBS: TRIGGERED" if msg.data else "EBS: ARMED", bg=self.color2 if msg.data else self.color5)

    def toggle_ebs_fail(self):
        msg = Bool()
        msg.data = not self.ebs_fail_button["text"].endswith("UNAVAILABLE")
        self.ebs_fail_pub.publish(msg)
        self.ebs_fail_button.config(text="EBS: UNAVAILABLE" if msg.data else "EBS: AVAILABLE", bg= self.color2 if msg.data else self.color5)

    def toggle_flag(self):
        msg = Bool()
        msg.data = not self.flag_button["text"].endswith("ON")
        self.flag_pub.publish(msg)
        self.flag_button.config(text="FLAG: ON" if msg.data else "FLAG: OFF", bg=self.color5 if msg.data else self.color2)

    def toggle_driving_flag(self):
        msg = Bool()
        msg.data = not self.driving_flag_button["text"].endswith("ON")
        self.driving_flag_pub.publish(msg)
        self.driving_flag_button.config(text="DRIVING FLAG: ON" if msg.data else "DRIVING FLAG: OFF", bg=self.color5 if msg.data else self.color2)


    def update(self):
        
        #state: AS OFF
        if self.state == 0:

            if self.ASmaster and self.TSmaster and self.EBS and self.misson  :
                self.state = 1
                self.CanStateMsg.as_state = 1
                self.canStatePub.publish(self.CanStateMsg)

        #state: AS Ready
        if self.state == 1: 
            self.node.get_logger().info(str(self.sdc))  
            if not self.ASmaster:
                self.state = 0  
                self.CanStateMsg.as_state = 0
                self.canStatePub.publish(self.CanStateMsg)  

            elif self.sdc == True:
                self.state = 3 
                self.CanStateMsg.as_state = 3
                self.canStatePub.publish(self.CanStateMsg)  

        #state: AS Driving
        elif self.state == 2:
            print("hi")
        
        #state: AS EBS
        elif self.state == 3:
            if not self.ASmaster:
                self.state = 0
        
        #state: AS Finished
        elif self.state == 4:  
            if not self.ASmaster:
                self.state = 0
                       
                





def main():
    root = tk.Tk()
    root.configure(bg = 'black')
    interface = Interface(root)
    
    root.mainloop()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
