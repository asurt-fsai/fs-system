from tkinter import *
import tkinter as tk
from threading import Thread
from PIL import ImageTk, Image
import rclpy
from std_msgs.msg import Float32

root = Tk()



screen1 = Frame(root, background='black')
screen2 = Frame(root, background='black')
root.title("Formula 1 AI")
root.geometry("1000x800") 

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


def show_screen1():
    screen1.place(relx=0, rely=0, relwidth=1, relheight=1)
    screen2.place_forget()

def show_screen2():
    screen1.place_forget()
    screen2.place(relx=0, rely=0, relwidth=1, relheight=1)
    root.geometry("1800x1200")

show_screen1()

def create_widgets_screen1():

    selectMissionLabel = Label(screen1, text="Select Mission", font=("aptos", 20, "bold"), background='black', fg="white")
    selectMissionLabel.place(relx=.5, rely=.2, anchor=CENTER)

    buttons = [
        ("Static A", .25, .35),
        ("Static B", .75, .35),
        ("Autocross", .25, .50),
        ("autoDemo", .75, .50),
        ("Skid Pad", .25, .65),
        ("Track Drive", .75, .65),
        ("Acceleration", .5, .8)
    ]

    for text, relx, rely in buttons:
        button = Button(screen1, text=text, font=("ubuntu", 12), background=color1, foreground=color2, activebackground=color3, activeforeground=color4, highlightbackground=color2, highlightthickness=2, highlightcolor='white', width=12, height=1, command=show_screen2)
        button.place(relx=relx, rely=rely, anchor=CENTER)


def create_widgets_screen2():


    labels = [
        ("Mission Type", .15, .20),
        ("AS state", .37, .20),
        ("SLAM loop closure", .62, .20),
        ("Lap Count", .85, .20),
        ("Velocity", .26, .33),
        ("Steering", .5, .33),
        ("Acceleration", .73, .33)
    ]

    for text, relx, rely in labels:
        label = Label(screen2, text=text, font=("aptos", 12, 'bold'),background='black', fg = 'white' )
        label.place(relx=relx, rely=rely, anchor=CENTER)

    
    missionlabel = tk.Label(screen2, text="autodemo", font=("aptos", 11), background='black', fg='white')
    missionlabel.place(relx=.15, rely=.25, anchor=tk.CENTER)

    stateLabel = tk.Label(screen2, text="driving", font=("aptos", 11), background='black', fg='white')
    stateLabel.place(relx=.37, rely=.25, anchor=tk.CENTER)

    loopLabel = tk.Label(screen2, text="N/A", font=("aptos", 11), background='black', fg='white')
    loopLabel.place(relx=.62, rely=.25, anchor=tk.CENTER)

    lapCountLabel = tk.Label(screen2, text="N/A", font=("aptos", 11), background='black', fg='white')
    lapCountLabel.place(relx=.85, rely=.25, anchor=tk.CENTER)
    
    velcityLabel = tk.Label(screen2, text="velocity", font=("aptos", 11), background='black', fg='white')
    velcityLabel.place(relx=.26, rely=.38, anchor=tk.CENTER)

    steeringLabel = tk.Label(screen2, text="Steering", font=("aptos", 11), background='black', fg='white')
    steeringLabel.place(relx=.5, rely=.38, anchor=tk.CENTER)

    accelerationLabel = tk.Label(screen2, text="Acceleration", font=("aptos", 11), background='black', fg='white')
    accelerationLabel.place(relx=.73, rely=.38, anchor=tk.CENTER)

    



create_widgets_screen1()
create_widgets_screen2()


root.mainloop()

