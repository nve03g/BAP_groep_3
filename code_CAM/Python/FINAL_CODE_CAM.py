"""
Created on Tue Mar 30 17:46:07 2021
Modified in 2023 by Péter Zoltán Csurcsia 
Modified in 2024 by Nellie Van Eeckhaute & Cloë Theys
@author: Pujan Bhandari
"""

""" Used libraries """
from tkinter import tix
import tkinter as tk
import cv2
import numpy as np

import time 
import imutils
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib import backend_bases
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from threading import Thread
import os
from csv import writer
import csv
import sys
import glob
import serial
import re
import configparser


# setting runtime configuration settings for matplotlib
plt.rcParams['savefig.dpi'] = 500
 #:'x-large'
plt.rcParams['ytick.labelsize']= 16
plt.rcParams['legend.fontsize']= 12
plt.rcParams['xtick.labelsize']= 16
plt.rcParams['font.size']= 10

backend_bases.NavigationToolbar2.toolitems = (
        ('Home', 'Reset original view', 'home', 'home'),
        ('Zoom', 'Zoom to rectangle', 'zoom_to_rect', 'zoom'),
        ('Save', 'Save the figure', 'filesave', 'save_figure'),
      )

""" Used global variables """
global I_switch # variable to check if the integral branch is turned on
global D_switch # variable to check if the derivative branch is turned on
global setpoint # variable to store the reference value
global tube_length # variable to store the length of the a beam(platform) as input form the user
global x_incm #variable to store the position of the ball when converted from pixels into centimeters

global prev_elapsedTime 
global prev_integ
global prev_deriv
global prev_PID

global openloop_switch
global deadtime # variable for the deadtime of the servomotor
global PID
global servo_signal # signal to send to the servomotor/signal to pass through the serial path
global com_port # neme of the com port where the arduino is connected
global error # variable for error signal
global prev_ball_pos # previous position of the ball in cm
#global I_windup_bool # variable to check if the path to avoid integral windup is activated
global LPF_bool # variable to check if the low pass filter is used
global D_kick_bool # variable to check if the bypass path to avoid derivative kick is activated
global FPS
global stepinput_bool
global ballpos_tocalibrate # helping variable to denote the position of the ball in pixels for minimum and the maximum possible range of the beam
global x_borderMax # maximum position where the ball can reach in pixels
global x_borderMin # minimum position where the ball can reach in pixels
global x_pos # helping variable to display in the label in GUI
global filename #variable for the name of the .csv file to store the measurements
global tf_factor # a factor which is used to multiply with the derivative time constant and use as the time constat for the low pass filter
global deriv 

global camera_connected_bool # helping variable to give feedback if camera is connected
global arduino_connected_bool # helping variable to give feedback if arduino is connected
global cam #variable for the name of the frame captured by the camera
global actual_fps # variable to calculate the actual FPS of the camera during operation


global camHeight # height of the frame 
global camWidth # width of the frame

""" Helping variables to make the deflection range of the servomotor adjustable (Vertical slider under the servomotor block)""" 
global initial_max_PWM
global initial_min_PWM
global calibrated_min_PWM
global calibrated_max_PWM
global min_PWM
global max_PWM
global neutral_angle

global lowerBound_ball_input # variable to save the lower HSV colour range of the ball
global upperBound_ball_input # variable to save the upper HSV colour range of the ball

global camera_no # camera number to use for the demonstration setup goes from 0, 1, 2....

global time_signaltofans # helping variable to check the interval to send the signal to the serial path 

global deadband_angle # variable to store the value of the deadband for the angle obtained from the slider in GUI
global deadband_error # variable to store the value of the deadband for the error signal obtained from the slider in GUI

global baudrate # variable to store the baudrate for serial communication
global comports # variable to store all available comports 

global l_h,l_s,l_v,u_h,u_s,u_v # helping variables to read each HSV values (upper and lower) from the config file

global config

global mem #scope memory length in seconds
global mem_changed #indiciate of the scope memory has been changed
global GraphThread_started #check if the scope thread started
global line1, line2
global start_timeFPS, nrFrame, exiting


""" initializing the variables"""
config = configparser.ConfigParser()

mem=60
mem_changed=0
start_timeFPS=0
nrFrame=0
exiting=False

GraphThread_started=False
previous_error=0.0
prev_elapsedTime = 0.0
timenow=0.0
pid_i =0.0
integ =0.0
deriv = 0.0
prev_integ = 0.0
prev_deriv = 0.0
integer_max = 0.0
PID = 0.0
servo_signal = 0.0
prev_PID = 0.0

error = 0.0
saturation = False
prev_ball_pos = 0
tf_factor = 0
FPS = 30 # setting the FPS at 30 as most USB cameras have 30FPS, this can be checked by using a online FPS checker website.
# 1920* 1080P
camHeight = 1080 #550 #suitable for the GUI
camWidth = 1980 #600 #suitable for the GUI
actual_fps = 0 # this will be used to see the effective FPS of the camera during operation
deadband = 0
time_signaltofans = 1 #default value for the deadtime of of the servomotor, this is adjustable in the GUI

deadband_angle = 0 # default value for the deadband on the angle going to the servo motor.
deadband_error = 0 # default value for the deadband on the error signal.

l_h = 0
l_s= 0
l_v= 0
u_h= 179
u_s= 255
u_v = 255

I_switch = 0
D_switch = 0
openloop_switch = 1

x_borderMax = 0
x_borderMin = 0
x_pos = 0

tube_length = 46.7
setpoint = tube_length/2

x_incm = 0.0
time_start  = time.time()

time_start_measinfo = time.time()
deadtime = time.time()

x_value = []
y_value = []
y_errorvalue = []
y_setpointvalue = []

initial_min_PWM = 0
initial_max_PWM = 255
calibrated_min_PWM = initial_min_PWM
calibrated_max_PWM = initial_max_PWM
min_PWM = initial_min_PWM
max_PWM = initial_max_PWM
neutral_angle = 0

camera_no=0
baudrate = 115200 # this value must match the boudrate that arduino uses for the serial communication
config = configparser.ConfigParser()
com_port =''

""" the main window of GUI"""
main_Window = tix.Tk() # tix is used because it allows placing information as tooltip for various objects in GUI

screenwidth = int(np.minimum(main_Window.winfo_screenwidth()-10,1900*1.5))
#screenwidth=1400
screenwidthratio=screenwidth/1900


buttondrift=4/0.23157894736842158*(1-screenwidthratio)
if screenwidthratio>1.1:
    buttondrift=0.8/0.23157894736842158*(1-screenwidthratio)
    


main_Window.title("Ball Balancing beam: PID controller") # Title of the main window
main_Window.geometry(str(screenwidth) +"x"+str(int(screenwidthratio*800))+"+0-50") # size of the main window for GUI starting from 5 pixels right and under the topleft corner of the screen
main_Window["bg"]="light blue" # background colour of the mainwindow, not visible in GUI because of background image
main_Window.resizable(0, 0) # the GUI is not resizable because of the placement of different objects in the GUI


main_Window.call("tk","scaling",screenwidthratio)


DPI_in_use=main_Window.winfo_fpixels('1i')
DPI_ratio=108/DPI_in_use


"""importing the background image(blockdiagram), resizing it to make it fit in the main window of GUI,
placing a label on the main window and placing the resized image in the label"""
my_pic = Image.open("Background_cam.png") 
resized = my_pic.resize((screenwidth,int(screenwidthratio*800)),Image.Resampling.LANCZOS)
bg = ImageTk.PhotoImage(resized,master = main_Window)
bg_lbl = tk.Label(main_Window,image = bg)
bg_lbl.place(x= 0,y = 0, relwidth = 1, relheight = 1)

""" Window for calibration of the camera"""
cameracalibration_window = tix.Toplevel(main_Window)
cameracalibration_window.geometry(str(int(screenwidthratio*480))+"x"+str(int(screenwidthratio*350)))
cameracalibration_window.title("Camera calibration")
cameracalibration_window["bg"]="white"
cameracalibration_window.resizable(0, 0) 
cameracalibration_window.withdraw()

""" video window to see the video from the camera for visual conformation, can be accessed from
the window for camera calibration"""
videoWindow = tk.Toplevel(cameracalibration_window)
videoWindow.title("Webcam Frame")
videoWindow.resizable(0, 0) 
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

""" Window to calibrate the servomotor and arduino """
systemcalibration_window = tix.Toplevel(main_Window)
systemcalibration_window.geometry(str(int(screenwidthratio*550))+"x"+str(int(screenwidthratio*450)))

systemcalibration_window.title("System calibration")
systemcalibration_window["bg"]="white"
systemcalibration_window.resizable(0, 0) 
systemcalibration_window.withdraw()

""" Imporing and resizing the images which will be placed to indicate if the branch is turned on or off"""
bg_open_horizontal = Image.open("open_horizontal.png")
bg_open_horizontal = bg_open_horizontal.resize((int(screenwidthratio*50),int(screenwidthratio*25)),Image.Resampling.LANCZOS)
bg_open_horizontal = ImageTk.PhotoImage(bg_open_horizontal, master = main_Window)

bg_closed_horizontal = Image.open("closed_horizontal.png")
bg_closed_horizontal = bg_closed_horizontal.resize((int(screenwidthratio*50),int(screenwidthratio*25)),Image.Resampling.LANCZOS)
bg_closed_horizontal = ImageTk.PhotoImage(bg_closed_horizontal, master = main_Window)

bg_open_vertical = Image.open("open_vertical.png")
bg_open_vertical = bg_open_vertical.resize((int(screenwidthratio*25),int(screenwidthratio*50)),Image.Resampling.LANCZOS)
bg_open_vertical = ImageTk.PhotoImage(bg_open_vertical, master = main_Window)

bg_closed_vertical = Image.open("closed_vertical.png")
bg_closed_vertical = bg_closed_vertical.resize((int(screenwidthratio*25),int(screenwidthratio*50)),Image.Resampling.LANCZOS)
bg_closed_vertical = ImageTk.PhotoImage(bg_closed_vertical, master = main_Window)

lbl_opencircuit_LPF= tk.Label(main_Window,image = bg_closed_horizontal,bg = "lightgreen")
lbl_opencircuit_LPF.place(x=screenwidthratio*950, y=screenwidthratio*363)

""" Setting default values for the sliders"""
setposDefalult = 0
sliderCoefKDefault = 0 
sliderCoefTiDefault = 0.001
sliderCoefTdDefault = 0 
sliderCoefTf_Default = 0 
slider_deadband_Default = 1

""" Placing different objects in the main window of the GUI at suitable place accordingly on the 
block diagram. Firstly all the objects is placed in the main window and the objects for system calibration and camera calibration is placed"""
# Sliders used
slider_setpos = tk.Scale(main_Window, from_=0, to=tube_length, orient="horizontal",bg = "white", length=screenwidthratio*150, width = screenwidthratio*15, resolution=0.1)
slider_setpos.set(setposDefalult)
slider_setpos.place(x=screenwidthratio*3,y = screenwidthratio*170)

sliderCoefK = tk.Scale(main_Window, from_=0, to=10, orient="horizontal",bg = "white", length=screenwidthratio*140, width = screenwidthratio*15, resolution=0.0001)
sliderCoefK.set(sliderCoefKDefault)
sliderCoefK.place(x=screenwidthratio*380,y = screenwidthratio*270)

sliderCoefTi = tk.Scale(main_Window, from_=0.01, to=5, orient="horizontal",bg = "white", length=screenwidthratio*200, width = screenwidthratio*15, resolution=0.001)
sliderCoefTi.set(sliderCoefTiDefault)
sliderCoefTi.place(x=screenwidthratio*655,y = screenwidthratio*160)

sliderCoefTd = tk.Scale(main_Window, from_=0, to=5, orient="horizontal", bg = "white",length=screenwidthratio*200, width =screenwidthratio* 15, resolution=0.001)
sliderCoefTd.set(sliderCoefTdDefault)
sliderCoefTd.place(x=screenwidthratio*655,y = screenwidthratio*301)

sliderCoefTf_factor = tk.Scale(main_Window, from_=0, to=5, orient="horizontal", bg = "lightgreen",length=screenwidthratio*200, width = screenwidthratio*15, resolution=0.001)
sliderCoefTf_factor.set(sliderCoefTf_Default)
sliderCoefTf_factor.place(x=screenwidthratio*900,y = screenwidthratio*495)

sliderCoefdeadband_error= tk.Scale(main_Window, from_=0, to=5, orient="horizontal", bg = "red",length=screenwidthratio*130, width =screenwidthratio* 15, resolution=0.1)
sliderCoefdeadband_error.set(slider_deadband_Default)
sliderCoefdeadband_error.place(x=screenwidthratio*230,y = screenwidthratio*270)

sliderdeadtime = tk.Scale(main_Window, from_=0, to=1, orient="horizontal", bg = "white",length=screenwidthratio*100, width = screenwidthratio*15, resolution=0.01)
sliderdeadtime.set(1)
sliderdeadtime.place(x =screenwidthratio*1645,y=screenwidthratio*45)


slidermaxPWM = tk.Scale(main_Window, from_=0, to=255, orient="horizontal", bg = "red",fg = 'white',length=screenwidthratio*100, width = screenwidthratio*15, resolution=1)
slidermaxPWM.set(255)
slidermaxPWM.place(x=screenwidthratio*1420,y =screenwidthratio*45)

sliderminPWM = tk.Scale(main_Window, from_=0, to=255, orient="horizontal", bg = "blue",fg='white',length=screenwidthratio*100, width = screenwidthratio*15, resolution=1)
sliderminPWM.set(1)
sliderminPWM.place(x =screenwidthratio*1310,y=screenwidthratio*45)


# Buttons used
Btn_StartPID = tk.Button(main_Window, text = "Start controller",bg = 'lightgreen', command=lambda:startPID())
Btn_StartPID.place(x=5, y=screenwidthratio*240)

Btn_Switch_I = tk.Button(main_Window, image = bg_open_horizontal, bg= "red", command = lambda:ISwitch())
Btn_Switch_I.place(x=screenwidthratio*550, y=screenwidthratio*218)

Btn_Switch_D = tk.Button(main_Window,image = bg_open_horizontal,bg= "red", command = lambda: DSwitch())
Btn_Switch_D.place(x=screenwidthratio*550, y=screenwidthratio*363)

Btn_Switch_Dervkick = tk.Button(main_Window,image = bg_open_vertical,bg= "red", command = lambda:d_kickfun() )
Btn_Switch_Dervkick.place(x=screenwidthratio*627-buttondrift, y=screenwidthratio*400)

Btn_Switch_LPF = tk.Button(main_Window, image = bg_open_vertical,bg= "red", command = lambda:LPF_fun() )
Btn_Switch_LPF.place(x=screenwidthratio*868-buttondrift, y=screenwidthratio*388)

Btn_Switch_closedloop = tk.Button(main_Window, image = bg_closed_vertical,bg= "lightgreen", command = lambda: openloop_switchfun())
Btn_Switch_closedloop.place(x=screenwidthratio*163-buttondrift, y=screenwidthratio*395)

Btn_systemcalibration_window = tk.Button(main_Window, text = "setup",bg = 'white', command=lambda:showsystemcalibration_window())
Btn_systemcalibration_window.place(x=screenwidthratio*1520, y=screenwidthratio*137)

Btn_cameracalibration_window = tk.Button(main_Window, text = "calibration",bg = 'white', command=lambda:showcameracalibration_window())
Btn_cameracalibration_window.place(x=screenwidthratio*850, y=screenwidthratio*740)


Btn_ResetIntergrator = tk.Button(main_Window, text = "R", command=lambda:resetIntegrator())
Btn_ResetIntergrator.place(x=screenwidthratio*820, y=screenwidthratio*220)
tip_Btn_ResetIntergrator = tix.Balloon(main_Window)
tip_Btn_ResetIntergrator.bind_widget(Btn_ResetIntergrator, balloonmsg = "Reset the integrator.") 


""" Frame where different buttons are packed together"""
FrameButtons = tk.LabelFrame(main_Window, text="Options", bg ='White')
FrameButtons.place(x=screenwidthratio*5,y=screenwidthratio*300,width=screenwidthratio*130)


Btn_reset = tk.Button(FrameButtons, text = "Reset controller",bg = 'white', command=lambda:reset_fun())
Btn_reset.pack(fill = "both")   

btn_startrec = tk.Button(FrameButtons,text = "Start recording",bg = 'white',command = lambda:start_rec())
btn_startrec.pack(fill = "both")

Btn_autoconnect = tk.Button(FrameButtons, text="Auto connect",bg = 'white', command=lambda:autoconnectfun())
Btn_autoconnect.pack(fill = "both")


""" Frame for the scope"""
FrameScope = tk.LabelFrame(main_Window, text="Scope", bg ='White')
FrameScope.place(x=screenwidthratio*970,y=screenwidthratio*550,width=screenwidthratio*190/DPI_ratio)

Btn_graph = tk.Button(FrameScope, text="Run",bg = 'white', relief="sunken", command=lambda:runningScopeWindow())
Btn_graph.grid(column = 0, row = 0, sticky = tk.W, padx=screenwidthratio*5)

lbl_mem = tk.Label(FrameScope,text="(s)",bg = 'white',fg = 'black')
lbl_mem.grid(column=2, row=0, sticky = tk.W)
txtinput_mem = tk.Entry(FrameScope, width=int(4*screenwidthratio))
txtinput_mem.insert(0, mem)
txtinput_mem.grid(column=1, row=0, sticky = tk.W)

def callback_mem(sv):
    global mem, mem_changed    
    if len(txtinput_mem.get())>0: 
        if mem < int(txtinput_mem.get()):            
            mem_changed=-1
        elif mem > int(txtinput_mem.get()):
            mem_changed=1            
        mem = int(txtinput_mem.get())        
    else:
        mem=10
    if mem<10:
        mem=10
        tk.messagebox.showwarning("Warning", "Minimum memory length is 10 seconds")
    elif mem>600:
        mem=600
        tk.messagebox.showwarning("Warning", "Maximum maximum length is 10 minutes")
    datatofile()
    save()



txtinput_mem.bind('<Return>', callback_mem)
txtinput_mem.bind('<FocusOut>', callback_mem)

""" Frame where the objects for taking inputs for step response experiment is packed together"""
Frame_stepinput = tk.LabelFrame(main_Window, text="Step input", bg ='White')
Frame_stepinput.place(x=screenwidthratio*5,y=screenwidthratio*80,width=screenwidthratio*130)

lbl_stepsize= tk.Label(Frame_stepinput,text="Step size",bg = 'white',fg = 'black')
lbl_stepsize.grid(row = 0,column = 0)

sv_step = tk.StringVar()
txtinput_stepinput = tk.Entry(Frame_stepinput,width = int(screenwidthratio*4.5),textvariable=sv_step)
txtinput_stepinput.grid(row = 0,column = 1)

Btn_step_input = tk.Button(Frame_stepinput, text = "Apply",bg = 'white', command =lambda:step_inputfun())
Btn_step_input.grid(row = 1,column = 0)

""" Frame where different labels are packed together, the background colour changes accorindgly to indicate is something is on or off"""
Frame_indicator = tk.LabelFrame(main_Window, text="Indicators", bg ='White')
Frame_indicator.place(x=screenwidthratio*200,y=screenwidthratio*30,width=screenwidthratio*160)

lbl_indi_controller= tk.Label(Frame_indicator,text="",bg = 'red',width = int(screenwidthratio*5))
lbl_indi_controller.grid(row = 0,column = 0)

lbl_indi_cont =  tk.Label(Frame_indicator,text="Controller",bg = 'white')
lbl_indi_cont.grid(row = 0,column = 1)

lbl_indi_arduino= tk.Label(Frame_indicator,text="",bg = 'red',width = int(screenwidthratio*5))
lbl_indi_arduino.grid(row = 1,column = 0)

lbl_indi_ard =  tk.Label(Frame_indicator,text="Arduino",bg = 'white')
lbl_indi_ard.grid(row = 1,column = 1)

lbl_indi_camera= tk.Label(Frame_indicator,text="",bg = 'red',width = int(screenwidthratio*5))
lbl_indi_camera.grid(row = 2,column = 0)

lbl_indi_cam =  tk.Label(Frame_indicator,text="Camera",bg = 'white')
lbl_indi_cam.grid(row = 2,column = 1)

lbl_indi_recording= tk.Label(Frame_indicator,text="",bg = 'red',width = int(screenwidthratio*5))
lbl_indi_recording.grid(row = 3,column = 0)

lbl_indi_rec =  tk.Label(Frame_indicator,text="Recording",bg = 'white')
lbl_indi_rec.grid(row = 3,column = 1)

"""label used """
lbl_PID_gegevens = tk.Label(main_Window,text="FPS  = 0",bg = 'white',fg = 'black')
lbl_PID_gegevens.place(x=screenwidthratio*850,y=screenwidthratio*710)

lbl_E = tk.Label(main_Window,text="E",bg = 'white',fg = 'black')
lbl_E.place(x=screenwidthratio*205, y=screenwidthratio*200)

lbl_E2 = tk.Label(main_Window,text="E",bg = 'white',fg = 'black')
lbl_E2.place(x=screenwidthratio*360, y=screenwidthratio*200)

lbl_POS = tk.Label(main_Window,text="PV",bg = 'white',fg = 'black')
lbl_POS.place(x=screenwidthratio*180, y=screenwidthratio*270)

lbl_KE = tk.Label(main_Window,text="KE",bg = 'white',fg = 'black')
lbl_KE.place(x=screenwidthratio*500, y=screenwidthratio*200)
lbl_KE2 = tk.Label(main_Window,text="KE",bg = 'white',fg = 'black')
lbl_KE2.place(x=screenwidthratio*1040, y=screenwidthratio*65)

lbl_KI = tk.Label(main_Window,text="KIE",bg = 'white',fg = 'black')
lbl_KI.place(x=screenwidthratio*1040, y=screenwidthratio*105)

lbl_C = tk.Label(main_Window,text="C",bg = 'white',fg = 'black')
lbl_C.place(x=screenwidthratio*1140, y=screenwidthratio*100)

lbl_MV = tk.Label(main_Window,text="MV",bg = 'white',fg = 'black')
lbl_MV.place(x=screenwidthratio*1300, y=screenwidthratio*100)

lbl_MVL = tk.Label(main_Window,text="MVL",bg = 'white',fg = 'black')
lbl_MVL.place(x=screenwidthratio*1600, y=screenwidthratio*100)

lbl_MVL2 = tk.Label(main_Window,text="MV-D",bg = 'white',fg = 'black')
lbl_MVL2.place(x=screenwidthratio*1750, y=screenwidthratio*100)

lbl_KD = tk.Label(main_Window,text="KED",bg = 'white',fg = 'black')
lbl_KD.place(x=screenwidthratio*1120, y=screenwidthratio*165)



""" Variables to give a tool-tip for selected objects"""
tip_Iswitch = tix.Balloon(main_Window)
tip_Dswitch = tix.Balloon(main_Window)
tip_D_kick = tix.Balloon(main_Window)
tip_LPF = tix.Balloon(main_Window)
tip_openloop = tix.Balloon(main_Window)

tip_tf_factor = tix.Balloon(main_Window)
tip_tf_factor.bind_widget(sliderCoefTf_factor, balloonmsg = "Chosen value will be multiplied with the "+"\n"+"timeconstant(Td) of derivative controller.")

tip_min_PWM = tix.Balloon(main_Window)
tip_min_PWM.bind_widget(sliderminPWM, balloonmsg = "This is the minimum PWM")

tip_deadband_error = tix.Balloon(main_Window)
tip_deadband_error.bind_widget(sliderCoefdeadband_error, balloonmsg = "Within this deadband, the output of the controller"+"\n"+"will be null.")

tip_deadtime = tix.Balloon(main_Window)
tip_deadtime.bind_widget(sliderdeadtime, balloonmsg = "This is the time difference between "+"\n"+"two PWM signals.")

tip_max_PWM = tix.Balloon(main_Window)
tip_max_PWM.bind_widget(slidermaxPWM, balloonmsg = "This is the maximum PWM")

""""dropdown menu for selecting manipulating operation from the signal manipulator block"""
manipulator_list = tk.StringVar()
manipulator_list.set("1")
drop_block = tk.OptionMenu(main_Window, manipulator_list, "1", "Δ", "Σ", "O")
drop_block.place(x=screenwidthratio*1210,y = screenwidthratio*120)

if screenwidthratio>1.4:
    drop_block.config(width=5)


""" Placing different objects in the window for system calibration """ 
Btn_arduino_refresh= tk.Button(systemcalibration_window, text = "Refresh com ports", command = lambda: refresh_comports())
Btn_arduino_refresh.place(x = screenwidthratio*200, y = screenwidthratio*20)


lbl_arduino_connect = tk.Label(systemcalibration_window,text="Arduino not connected",bg = 'red',fg = 'black')
lbl_arduino_connect.place(x = screenwidthratio*200, y = screenwidthratio*60)

lbl_extra_info = tk.Label(systemcalibration_window,text="Please connect your Arduino device and experiment with the PWM values."+"\n"+"During the webcam calibration process the minimum value and the actual"+"\n" +"maximum applied value (i.e., the blue slider) will be automatically set."+"\n"+"Later on you can readjust these values manually if it is needed."+"\n" +"\n" +"Provide PWM values in the range [0,255].",bg = 'white',fg = 'black', justify=tk.LEFT)
lbl_extra_info.place(x= screenwidthratio*20,y = screenwidthratio*230)

lbl_PWM_to_fans = tk.Label(systemcalibration_window,text="check" +"\n" + "PWM value",bg = 'white',fg = 'black')
lbl_PWM_to_fans.place(x= screenwidthratio*300,y = screenwidthratio*120)

sv_checkPWM = tk.StringVar()
txtinput_PWM_to_fans = tk.Entry(systemcalibration_window,width = int(screenwidthratio*10),textvariable=sv_checkPWM)
txtinput_PWM_to_fans.place(x= screenwidthratio*385,y = screenwidthratio*120)

btn_applyangle = tk.Button(systemcalibration_window,text = "Apply",command = lambda:signal_to_fans())
btn_applyangle.place(x = screenwidthratio*385,y = screenwidthratio*145)

lbl_min_PWM = tk.Label(systemcalibration_window,text="minimim" +"\n" +"PWM value = ",bg = 'white',fg = 'black')
lbl_min_PWM.place(x= screenwidthratio*20,y = screenwidthratio*120)


sv_min_PWM = tk.StringVar()
txtinput_min_PWM = tk.Entry(systemcalibration_window,textvariable=sv_min_PWM)
txtinput_min_PWM.pack()
txtinput_min_PWM.place(x= screenwidthratio*130,y = screenwidthratio*140)

lbl_max_PWM = tk.Label(systemcalibration_window,text="maximum" +"\n" +"PWM value = ",bg = 'white',fg = 'black')
lbl_max_PWM.place(x= screenwidthratio*20,y = screenwidthratio*170)

Btn_arduino= tk.Button(systemcalibration_window, text = "Connect arduino", command = lambda: connect_arduinofun())
Btn_arduino.place(x = screenwidthratio*20, y = screenwidthratio*60)


sv_max_PWM = tk.StringVar()
txtinput_max_PWM = tk.Entry(systemcalibration_window,textvariable=sv_max_PWM)
txtinput_max_PWM.pack()
txtinput_max_PWM.place(x= screenwidthratio*130,y = screenwidthratio*190)

Btn_applyall= tk.Button(systemcalibration_window, text = "Apply", command = lambda:applyall_fun())
Btn_applyall.place(x = screenwidthratio*150, y = screenwidthratio*370)

Btn_finish_systemcalibration = tk.Button(systemcalibration_window, text = "OK",bg = 'white', command=lambda:finish_systemcalibration())
Btn_finish_systemcalibration.place(x = screenwidthratio*20, y = screenwidthratio*370)

# balloons to pop up over a widget and provide users descriptive messages
tip_min_PWM = tix.Balloon(systemcalibration_window)
tip_min_PWM.bind_widget(txtinput_min_PWM, balloonmsg = "Enter the minimum PWM value" +"\n" +"for the fan to lift the ball.")

tip_max_PWM = tix.Balloon(systemcalibration_window)
tip_max_PWM.bind_widget(txtinput_max_PWM, balloonmsg = "Enter the maximum PWM value for the fan" +"\n" +"to keep the ball inside the tube")

tip_btn_applyangle = tix.Balloon(systemcalibration_window)
tip_btn_applyangle.bind_widget(btn_applyangle, balloonmsg = "Send given PWM value to the fan")

def callback_checkPWM(sv):
    signal_to_fans()

def callback_stepinput(sv):
    step_inputfun()
    

def callback_min_PWM(sv):
    global txtinput_min_PWM
    sliderminPWM.set(int(txtinput_min_PWM))

def callback_max_PWM(sv):
    global txtinput_max_PWM
    slidermaxPWM.set(int(txtinput_max_PWM))
    

def callback_update(event):
    global K, Ti, Td, tf_factor, Tf, deadband_angle, deadband_error, max_PWM, min_PWM, time_signaltofans, setpoint
   
    K = sliderCoefK.get()
    Ti = sliderCoefTi.get()
    Td = sliderCoefTd.get()
    tf_factor = sliderCoefTf_factor.get()
    Tf = tf_factor * Td

    setpoint = slider_setpos.get()
    
    deadband_angle = 0
    deadband_error = sliderCoefdeadband_error.get()
    max_PWM = int(slidermaxPWM.get())
    min_PWM = int(sliderminPWM.get())
    
    time_signaltofans = sliderdeadtime.get()
    
    datatofile()
    save()


# keybindings: 
#    <Return> = enter 
#    <FocusOut> = keyboard focus moved from this widget to another widget
#    <ButtonRelease> = when button is released
txtinput_PWM_to_fans.bind('<Return>', callback_checkPWM)
txtinput_stepinput.bind('<Return>', callback_stepinput)

txtinput_min_PWM.bind('<Return>', callback_min_PWM)
txtinput_min_PWM.bind('<FocusOut>', callback_min_PWM)

txtinput_max_PWM.bind('<Return>', callback_max_PWM)
txtinput_max_PWM.bind('<FocusOut>', callback_max_PWM)


sliderCoefK.bind('<ButtonRelease>', callback_update)
sliderCoefTi.bind('<ButtonRelease>', callback_update)
sliderCoefTd.bind('<ButtonRelease>', callback_update)
sliderCoefTf_factor.bind('<ButtonRelease>', callback_update)
sliderminPWM.bind('<ButtonRelease>', callback_update)
sliderCoefdeadband_error.bind('<ButtonRelease>', callback_update)
slidermaxPWM.bind('<ButtonRelease>', callback_update)

slider_setpos.bind('<ButtonRelease>', callback_update)



"""Placing different objects on the window for camera calibration"""
lbl_camera_no = tk.Label(cameracalibration_window,text="Camera no. = ",bg = 'white',fg = 'black')
lbl_camera_no.place(x = screenwidthratio*20, y = screenwidthratio*20)

txtinput_camera_no = tk.Entry(cameracalibration_window, width=int(4*screenwidthratio))
txtinput_camera_no.place(x = screenwidthratio*160, y = screenwidthratio*20)
txtinput_camera_no.insert(0,camera_no)


Btn_setup_HSV = tk.Button(cameracalibration_window, text = "Open HSV Window", command = lambda:openHSV_window())
Btn_setup_HSV.place(x = screenwidthratio*20, y = screenwidthratio*80)


Btn_camera = tk.Button(cameracalibration_window, text = "Connect camera",command = lambda: connect_camerafun())
Btn_camera.place(x = screenwidthratio*200, y = screenwidthratio*80)

lbl_camera_connect = tk.Label(cameracalibration_window,text="camera not connected",bg = 'red',fg = 'white')
lbl_camera_connect.place(x = screenwidthratio*200, y = screenwidthratio*20)

Btn_ShowVideo = tk.Button(cameracalibration_window, text="Show video", command= lambda:showCameraFrameWindow())
Btn_ShowVideo.place(x = screenwidthratio*360, y = screenwidthratio*80)

lbl_info = tk.Label(cameracalibration_window,text="1) Perform HSV calibration (camera connects automatically)"+"\n"+"2) Enter the height of the tube"+"\n"+"3) Set minimum and maximum position"+"\n"+"4) Click OK",bg = 'white',fg = 'black', justify=tk.LEFT)
lbl_info.place(x=screenwidthratio*5,y=screenwidthratio*120)

lbl_beamlength = tk.Label(cameracalibration_window,text="Tube's height [cm]= ",bg = 'white',fg = 'black')
lbl_beamlength.place(x=screenwidthratio*20,y=screenwidthratio*200)

txtinput_beamlength = tk.Entry(cameracalibration_window, width=int(5*screenwidthratio))
txtinput_beamlength.place(x= screenwidthratio*160,y = screenwidthratio*200 )

lbl_posinfo = tk.Label(cameracalibration_window,text="",bg = 'white',fg = 'black')
lbl_posinfo.place(x=screenwidthratio*250,y=screenwidthratio*250)

Btn_setminpos = tk.Button(cameracalibration_window, text = "set minpos", command = lambda:set_minposfun() )
Btn_setminpos.place(x =screenwidthratio* 20, y = screenwidthratio*250)

Btn_setmaxpos = tk.Button(cameracalibration_window, text = "set maxpos", command = lambda:set_maxposfun() )
Btn_setmaxpos.place(x =screenwidthratio* 140, y =screenwidthratio* 250)

Btn_finish_cameracalibration = tk.Button(cameracalibration_window, text = "OK",bg = 'white', command=lambda:finish_cameracalibration())
Btn_finish_cameracalibration.place(x =screenwidthratio*20, y = screenwidthratio*290)





""" Here ends the construction of the GUI. The different functions used are constructed and assigned to the objects above. """
""" Function to read all the inputs from the system calibration window"""
def applyall_fun():
    global initial_max_PWM
    global initial_min_PWM
    global calibrated_min_PWM, calibrated_max_PWM
    try:
        calibrated_min_PWM = int(txtinput_min_PWM.get())
        calibrated_max_PWM = int(txtinput_max_PWM.get())
        sliderminPWM.set(calibrated_min_PWM)
        slidermaxPWM.set(calibrated_max_PWM)
        datatofile()
        save()
    except:
        tk.messagebox.showerror("Error","Error while entering calibration parameters") 
        
""" Function used for resetting the controller. All the sliders are set to its default value."""  
def reset_fun():
    global setpoint
    global x_pos
    global neutral_angle
    global PID
    global integ
    global deriv
    global prev_PID
    global prev_integ
    global prev_deriv
    global error 
    global previous_error
    global elapsedTime
    global prev_ball_pos
    global prev_elapsedTime
    global mem, mem_changed, runningScope
    
    previous_error = 0
    prev_elapsedTime = 0
    prev_integ = 0
    prev_deriv = 0
    prev_PID = 0
    prev_ball_pos = 0
    error = 0
    integ = 0
    deriv = 0
    mem=60
    mem_changed=1
    runningScope=1
    
    slider_setpos.set(x_pos)
    sliderCoefK.set(sliderCoefKDefault)
    sliderCoefTi.set(sliderCoefTiDefault)
    sliderCoefTd.set(sliderCoefTdDefault) 
    sliderCoefTf_factor.set(sliderCoefTf_Default)
    sliderCoefdeadband_error.set(slider_deadband_Default)
    txtinput_mem.delete(0,tk.END)
    txtinput_mem.insert(0, mem)
    
    runningScopeWindow()
    
    try:
        ser.write(str(chr(int(neutral_angle))).encode())
        ser.flush()
    except:
        pass


"""Function to finish system calibration and close the calibration window"""
def finish_systemcalibration():
    global show_systemcalibration_window
    global min_PWM
    global max_PWM
    global initial_max_PWM
    try:
        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "setup"
    except:
        tk.messagebox.showerror("showerror","Error while improting inputs") 
        
"""Function to finish camera calibration and close the calibration window"""
def finish_cameracalibration():
    global tube_length
    global show_cameracalibration_window

    try:
        cameracalibration_window.withdraw()
        show_cameracalibration_window = False
        Btn_cameracalibration_window["text"] = "calibration"
        slider_setpos.set(tube_length/2)

    except:
        tk.messagebox.showerror("showerror","Error while reading inputs")   
        
"""Function to open the window to extract the HSV colour range of the ball"""
def openHSV_window():
    global camera_no, camera_connected_bool
    try:
        camera_no = int(txtinput_camera_no.get())
        if camera_connected_bool==False:
            autoconnectfun()
            connect_camerafun()
        if camera_connected_bool==True:
            run_all(camera_no) # this function calls another function which is retrieved from an open source example code. 
            datatofile()
            save()
        else:
            lbl_camera_connect.config(text = "Camera not connected")
            lbl_camera_connect["bg"] = "red"
            lbl_indi_camera["bg"] = 'red'
    except:
        tk.messagebox.showerror("showerror","Error while taking camera number.")

"""Function to open the camera calibration window"""
show_cameracalibration_window = False
def showcameracalibration_window():
    global show_cameracalibration_window
    if show_cameracalibration_window == False:
        cameracalibration_window.deiconify()
        show_cameracalibration_window = True
        Btn_cameracalibration_window["text"] = "Close"
    else:
        cameracalibration_window.withdraw()
        show_cameracalibration_window = False
        Btn_cameracalibration_window["text"] = "Calibration"

def ErrorArduino():
    tk.messagebox.showerror("Error","Error connecting to the arduino")  
    
"""Function to connect the hardware using the previous calibration data stored in the configuration file"""
def autoconnectfun():
    global FPS
    global camera_no
    global tube_length
    global x_borderMin
    global x_borderMax
    global lowerBound_ball_input
    global upperBound_ball_input
    global neutral_angle
    global min_PWM
    global max_PWM
    global time_signaltofans
    global baudrate
    global com_port
    global deadband_error
    global ser
    global deadband_angle
    global l_h ,l_s,l_v ,u_h,u_s,u_v
    global initial_min_PWM
    global initial_max_PWM
    global mem, runningScope
    global K, Ti, Td, tf_factor, Tf
    global I_switch, I_switch_bool, D_switch, D_switch_bool, D_kick_bool, LPF_bool, openloop_switch_bool, openloop_switch, mem_changed

    
    try:
        config.read('config.INI')
        
        camera_no = int(config['Camera']['camera_no'])    
        txtinput_camera_no.delete(0, 'end') 
        txtinput_camera_no.insert(0,camera_no)
        
        tube_length = float(config['Camera']['platforms_length'])
        txtinput_beamlength.delete(0, 'end') 
        txtinput_beamlength.insert(0,tube_length) 
        
        x_borderMin = float(config['Camera']['pix_minpos'])
        x_borderMax = float(config['Camera']['pix_maxpos'])
        
        l_h  = int(config['Camera']['l_h'])   
        l_s = int(config['Camera']['l_s'])
        l_v = int(config['Camera']['l_v'])
        u_h  = int(config['Camera']['u_h'])
        u_s = int(config['Camera']['u_s'])
        u_v = int(config['Camera']['u_v'])      
        lowerBound_ball_input = np.array ([l_h,l_s,l_v])
        upperBound_ball_input = np.array([u_h,u_s,u_v])

        neutral_angle = 0
        txtinput_PWM_to_fans.delete(0, 'end') 
        txtinput_PWM_to_fans.insert(0,neutral_angle) 
        
        txtinput_min_PWM.delete(0, 'end') 
        txtinput_min_PWM.insert(0,initial_min_PWM)
    
        txtinput_max_PWM.delete(0, 'end') 
        txtinput_max_PWM.insert(0,initial_max_PWM)
        
        max_PWM = int(config["System"]['maximum_PWM'])
        slidermaxPWM["from"] = initial_min_PWM
        slidermaxPWM["to"] = initial_max_PWM
        slidermaxPWM.set(max_PWM)

        min_PWM = int(config["System"]['minimum_PWM'])
        sliderminPWM["from"] = initial_min_PWM
        sliderminPWM["to"] = initial_max_PWM
        sliderminPWM.set(min_PWM)
    
        time_signaltofans = float(config['System']['deadtime_fans'])    
        baudrate = int(config['System']['baudrate'])
        com_port = (config['System']['com_port'])
        deadband_error = float(config['PID']['deadband_error'])
        deadband_angle = float(config['PID']['deadband_angle'])
        
        manipulator_list.set(config['PID']['manipulator'])
        mem= int(config['Scope']['mem'])    
        
        
        runningScope= bool(config['Scope']['runningscope'])          
        runningScopeWindow_plot()
        
        
        K = float(config['PID']['k'])
        Ti = float(config['PID']['ti'])
        Td = float(config['PID']['td'])
        tf_factor = float(config['PID']['tf_factor'])
        Tf = tf_factor * Td
                        
        txtinput_mem.delete(0, 'end') 
        txtinput_mem.insert(0, mem)
        
        sliderCoefK.set(K)
        sliderCoefTi.set(Ti)
        sliderCoefTd.set(Td)
        sliderCoefTf_factor.set(tf_factor)
        
 
        sliderdeadtime.set(time_signaltofans)
        sliderCoefdeadband_error.set(deadband_error)
        slider_setpos["to"] = tube_length
        slider_setpos.set(tube_length/2)
        
             
        
        I_switch=int(config['PID']['i_active'])
        I_switch_bool=bool(I_switch)
        
        D_switch=int(config['PID']['d_active'])
        D_switch_bool=bool(D_switch)
        
        
        D_kick_bool=config['PID']['d_kick_active']=='True'
        LPF_bool=config['PID']['lpf_filter']=='True'
        openloop_switch_bool=config['PID']['open_loop']=='True'
        openloop_switch=int(openloop_switch_bool)
        
        ISwitch_GUI()  
        DSwitch_GUI()
        d_kickfun_GUI()
        LPF_fun_GUI()
        openloop_switchfun_GUI()
        
        mem_changed=1
        
        
        try:
            threadCAM=Thread(target=connect_camerafun)
            threadCAM.setDaemon(True)
            threadCAM.start()                        
        except:
            pass  
        try:    
            threadArduino=Thread(target=connect_arduinofun)
            threadArduino.setDaemon(True)
            threadArduino.start()                        
        except:
            threadError=Thread(target=ErrorArduino)
            threadError.setDaemon(True)
            threadError.start()                                       
    except:
        tk.messagebox.showerror("Error","No valid configuration file has been found.")  
    
""" Function to refresh all the comports available and update the dropdown menu in the system calibration window.
The function is adapted from:
“Serial List Ports in Python - Serial_ports().” Gist,
https://gist.github.com/tekk/5c8d6824108dfb771a1e16aa8bc479f0.

Accessed 25 March 2021.

"""
def refresh_comports():
    global manipulator_list_comport
    global ser
    try:
        
        ser.close()
        def serial_ports():
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')

            result = []
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    result.append(port)
                except (OSError, serial.SerialException):
                    pass
            return result
        comports= serial_ports()
        manipulator_list_comport = tk.StringVar()
        manipulator_list_comport.set("Choose a com port")
        drop_block_comport = tk.OptionMenu(systemcalibration_window, manipulator_list_comport, *comports)
        drop_block_comport.place(x=screenwidthratio*20,y = screenwidthratio*20)
    except:
        def serial_ports():
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')

            result = []
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    result.append(port)
                except (OSError, serial.SerialException):
                    pass
            return result

        comports= serial_ports()
        manipulator_list_comport = tk.StringVar()
        manipulator_list_comport.set("Choose a com port")
        try:            
            drop_block_comport = tk.OptionMenu(systemcalibration_window, manipulator_list_comport, *comports)
            drop_block_comport.place(x= screenwidthratio*20,y = screenwidthratio*20)
        except:
            lbl_arduino_connect.config(text = "No arduino found.")


     
"""Function to display calibration window for arduino and servo motor."""
show_systemcalibration_window = False
def showsystemcalibration_window():
    global show_systemcalibration_window
    global comports
    if show_systemcalibration_window == False:
        systemcalibration_window.deiconify()
        show_systemcalibration_window = True
        Btn_systemcalibration_window["text"] = "Close"

    else:
        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "Calibration"

"""Function to display the video from the camera."""
showVideoWindow = False
def showCameraFrameWindow():
    global showVideoWindow
    global BRetourVideoTxt
    global lowerBound_ball_input
    global upperBound_ball_input
    global camera_connected_bool
        
    if camera_connected_bool==False:
        connect_camerafun()
    if camera_connected_bool==True:        
        if showVideoWindow == False:
            try:            
                videoWindow.deiconify()
                showVideoWindow = True
                Btn_ShowVideo["text"] = "Close Video "
               
            except:
                videoWindow.deiconify()
                showVideoWindow = True
                Btn_ShowVideo["text"] = "Close Video "
        else:
            videoWindow.withdraw()
            showVideoWindow = False
            Btn_ShowVideo["text"] = "Show video"
        

"""Function to display the graph in real time. The graph is updated each second. This can be improved"""
runningScope = True
def runningScopeWindow_plot():
    global runningScope
    if runningScope == True:
        Btn_graph.config(relief="sunken")        
    else:        
        Btn_graph.config(relief="raised")
        
        
def runningScopeWindow():
    global runningScope, mem_changed, canvas
    if runningScope == False:
        runningScope = True
    else:
        runningScope = False     
        canvas.draw()
        canvas.flush_events()   
    runningScopeWindow_plot()


"""Function to read the camera number, read the lower and upper HSV colour range, connect the camera and give proper indication"""
camera_connected_bool = False
def connect_camerafun():
    global camera_connected_bool
    global FPS
    global cam
    global camHeight
    global camWidth
    global camera_no
    global lowerBound_ball_input
    global upperBound_ball_input
    global config
    global showVideoWindow
    
    try:
        camera_no = int(txtinput_camera_no.get())
        # to be tested
        #  CAP_ANY CAP_OPENCV_MJPEG CAP_DSHOW
        cam = cv2.VideoCapture(camera_no,cv2.CAP_ANY)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        #to be tested
        
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        
        if not cam.isOpened():
            tk.messagebox.showerror("Error","Error connecting to the webcam")
            camera_connected_bool = False
            exit()
        
        cam.set(cv2.CAP_PROP_FRAME_WIDTH,camWidth)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT,camHeight)
        
        
        # to be tested
        #cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))

        # to be tested
        #cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera

        #cam.set(cv2.CAP_PROP_FPS, 100)
        camera_connected_bool = True
        # print("just before opening configfile.")
        config.read('config.INI')
        # print("reading config file")
        l_h  = int(config['Camera']['l_h'])   
        # print(l_h)                         
        l_s = int(config['Camera']['l_s'])
        # print(type(l_s))
        l_v = int(config['Camera']['l_v'])
        u_h  = int(config['Camera']['u_h'])
        u_s = int(config['Camera']['u_s'])
        u_v = int(config['Camera']['u_v'])        
        lowerBound_ball_input = np.array ([l_h,l_s,l_v])
        upperBound_ball_input = np.array([u_h,u_s,u_v])
        lbl_camera_connect.config(text = "camera is connected")
        lbl_indi_camera["bg"] = 'lightgreen'
        lbl_camera_connect["bg"] = "lightgreen"
        main()
        
    except:
        lbl_camera_connect.config(text = "Camera connection failed")
        lbl_camera_connect["bg"] = "red"
        lbl_indi_camera["bg"] = 'red'
        
"""Function to read the selected com port and connect the arduino"""
arduino_connected_bool = False
def connect_arduinofun():
    global ser
    global arduino_connected_bool
    global baudrate
    global com_port
    global manipulator_list_comport
    global neutral_angle
    try:
        try:
            com_selection =str( manipulator_list_comport.get())
            com_port = re.sub('\W', '', com_selection)             
        except:
            pass
        ser = serial.Serial(com_port,baudrate) 

        ser.write(str(chr(int(neutral_angle))).encode())
        ser.flush()
        
        
        arduino_connected_bool = True
        lbl_arduino_connect.config(bg = "lightgreen")
        lbl_arduino_connect.config(text = "Arduino connected.")
        lbl_indi_arduino.config(bg = "lightgreen")

        
    except:
        lbl_arduino_connect.config(bg = "red")
        lbl_arduino_connect.config(text = "Arduino connection failed.")
        lbl_indi_arduino["bg"] = 'red'
        
                    
    

"""Function to indicate that controller is started so that the main code calls the function where the controller is implemented """
Start_PID = False
def startPID():
    global Start_PID
    global FPS
    global tube_length
    global tf_factor
    global setpoint
    global xincm
    global ser
    global neutral_angle
    if Start_PID == False:
        if arduino_connected_bool == False:
            tk.messagebox.showerror("PID controller cannot start","There is no arduino connected.")      
        elif camera_connected_bool == True:
            Start_PID = True
            Btn_StartPID["text"] = "Stop Controller"
            Btn_StartPID['bg'] = "red"
            lbl_indi_controller["bg"] = "lightgreen"
            Btn_StartPID.relief="sunken"
        else:
            tk.messagebox.showerror("PID controller cannot start","There is no camera connected.")      
    else:
        Start_PID = False
        Btn_StartPID["text"] = "Start Controller"
        Btn_StartPID['bg'] = "lightgreen"
        lbl_indi_controller["bg"] = "red"
        Btn_StartPID.relief="raised"
        try:
            ser.write(str(chr(neutral_angle)).encode())
            ser.flush()
            
        except:
            pass   


"""The following functions are used to indicate if any particular branch is turned on, the boolean variable and the indicators are adjusted accordingly"""
I_switch_bool = False
tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn on the integrator.") 

def ISwitch_GUI():
    global I_switch_bool
    if I_switch_bool == True:
        tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn off the integrator.") 
        Btn_Switch_I["image"] = bg_closed_horizontal
        Btn_Switch_I['bg'] = "lightgreen"
    else:
        tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn on the integrator.") 
        Btn_Switch_I["image"] =bg_open_horizontal
        Btn_Switch_I['bg'] = "red"


def ISwitch():
    global I_switch
    global I_switch_bool
    if I_switch_bool == False:
        I_switch = 1
        I_switch_bool = True
    else:
        I_switch = 0
        I_switch_bool = False
        tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn on the integrator.") 

    ISwitch_GUI()        
    datatofile()
    save()

openloop_switch_bool = True
tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a openloop system.")


"""Fuction to connect and disconnect the feedback branch of the closed loop system"""
def openloop_switchfun_GUI():
    global openloop_switch_bool
    if openloop_switch_bool == False:
        tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a closedloop system.")
        Btn_Switch_closedloop["image"] = bg_open_vertical
        Btn_Switch_closedloop['bg'] = "red"
    else:
        tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a openloop system.")
        Btn_Switch_closedloop["image"] = bg_closed_vertical
        Btn_Switch_closedloop['bg'] = "lightgreen"
    
def openloop_switchfun():
    global openloop_switch
    global openloop_switch_bool
    if openloop_switch_bool == True:
        openloop_switch = 0
        openloop_switch_bool = False
    else:
        openloop_switch = 1
        openloop_switch_bool = True
    openloop_switchfun_GUI()
    datatofile()
    save()


D_switch_bool = False
tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
def DSwitch_GUI():
    global D_switch_bool
    if D_switch_bool == True:
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn off the derivative.") 
        Btn_Switch_D["image"] = bg_closed_horizontal
        Btn_Switch_D['bg'] = "lightgreen"
        
        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Dervkick["image"] = bg_open_vertical
        Btn_Switch_Dervkick["bg"] = "red"
        
    else:
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
        Btn_Switch_D["image"] = bg_open_horizontal
        Btn_Switch_D['bg'] = "red"   

def DSwitch():
    global D_switch
    global D_switch_bool
    global LPF_bool
    global D_kick_bool
    if D_switch_bool == False:
        D_switch = 1
        D_switch_bool = True        
        D_kick_bool = False        
    else:
        D_switch = 0
        D_switch_bool = False
    DSwitch_GUI()
    datatofile()
    save()
             
D_kick_bool = False
tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")

def d_kickfun_GUI():
    global D_switch
    global D_switch_bool
    global D_kick_bool
    global tf_factor
    if D_kick_bool == True:
        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to disconnect this branch.")
        Btn_Switch_Dervkick["image"] = bg_closed_vertical
        Btn_Switch_Dervkick["bg"] = "lightgreen"

        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
        Btn_Switch_D["image"] = bg_open_horizontal
        Btn_Switch_D['bg'] = "red"
    else:
        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Dervkick["image"] = bg_open_vertical
        Btn_Switch_Dervkick["bg"] = "red"


def d_kickfun():
    global D_switch
    global D_switch_bool
    global D_kick_bool
    global tf_factor
    if D_kick_bool == False:
        D_kick_bool = True
        D_switch = 1
        D_switch_bool = False
    else:
        D_kick_bool = False
        D_switch = 0
    d_kickfun_GUI()
    datatofile()
    save()
 
LPF_bool = False
tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to activate low pass filter.")

def LPF_fun_GUI():
    global LPF_bool
    if LPF_bool == True:

        tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to deactivate low pass filter.")
        Btn_Switch_LPF["image"] = bg_closed_vertical
        Btn_Switch_LPF['bg'] = "lightgreen"
        lbl_opencircuit_LPF["image"] = bg_open_horizontal
        lbl_opencircuit_LPF["bg"] = 'red'
    else:

        tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to activate low pass filter.")
        Btn_Switch_LPF["image"] = bg_open_vertical
        Btn_Switch_LPF['bg'] ="red"
        lbl_opencircuit_LPF["image"] = bg_closed_horizontal
        lbl_opencircuit_LPF["bg"] = "lightgreen"

def LPF_fun():
    global D_switch
    global D_switch_bool
    global LPF_bool
    global D_kick_bool
    global tf_factor
    if LPF_bool == False:
        LPF_bool = True
    else:
        LPF_bool = False
    LPF_fun_GUI()
    datatofile()
    save()

"""Function to take stepsize from the user and set the new value of setpoint"""
stepinput_bool = False
def step_inputfun():
    global setpoint
    try:
        step = float(txtinput_stepinput.get())
        nieuw_setpoint = setpoint + step
        slider_setpos.set(nieuw_setpoint)        
    except:
        tk.messagebox.showerror("showerror","Error while taking inputs from user.")      
    
"""Following two functions are used to take the minimum and maximum value of pixels where the ball 
can fly up and down. These values are used to convert the pixels into centimeter in the main function"""
def set_minposfun():
    global x_borderMin, x_borderMax
    global ballpos_tocalibrate
    global tube_length, mem_changed
    try:
        
        mem_changed=1
        tube_length = float(txtinput_beamlength.get())
        x_borderMin = ballpos_tocalibrate
        lbl_posinfo.config(text = "min position is taken")
        
        if x_borderMin>x_borderMax and x_borderMax>0:
            tk.messagebox.showwarning("Warning", "Your minimum position is greater than the maximum position")
        
    except:
        lbl_posinfo.config(text = "Error")
 
def set_maxposfun():
    global x_borderMax, x_borderMin
    global ballpos_tocalibrate
    global tube_length, mem_changed
    
    try:
        mem_changed=1
        tube_length = float(txtinput_beamlength.get())
        slider_setpos["to"] = tube_length
        x_borderMax = ballpos_tocalibrate 
        lbl_posinfo.config(text = "max position is taken")
        
        if x_borderMax<x_borderMin:
            tk.messagebox.showwarning("Warning", "Your maximum position is smaller than the minimum position")

    except:
        lbl_posinfo.config(text = "Error")
        
"""Function to call when the start/stop recording button is manipulator_list"""
start_rec_bool = False
def start_rec():
    global start_rec_bool
    global filename
    if start_rec_bool == False:
        start_rec_bool = True
        lbl_indi_recording["bg"] = "lightgreen"
        btn_startrec["text"] = "Stop recording"
        filename = time.strftime('%Y%m%d-%H%M%S') 
        filename = "data"+filename +".csv"
        with open(filename, 'w', newline='') as write_obj:
            csv_writer = writer(write_obj,delimiter=';')
            fieldname = ['time','setpoint','error','controller output', 'manipulated variable','process value']
            csv_writer = csv.DictWriter(write_obj,fieldnames = fieldname,delimiter = ';')
            csv_writer.writeheader()
    else:
        start_rec_bool = False
        btn_startrec["text"] = "Start recording"
        lbl_indi_recording["bg"] = "red"

"""Function used to send an angle to the servomotor to get a proper maximum and minimum angle
 of the servomotor"""
def signal_to_fans():
    global ser
    try:
        sent_PWM_signal = int(txtinput_PWM_to_fans.get()) 

        ser.write(str(chr(int(sent_PWM_signal))).encode())
        ser.flush()
        
    except:
        tk.messagebox.showerror("showerror","Error sending signal to arduino")       
        
"""function to call to update/save a .csv file"""
def append_list_as_row(file_name, list_of_elem): 
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj,delimiter=';')
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)
    
def resetSlider():
    sliderCoefK.set(sliderCoefKDefault)
    sliderCoefTi.set(sliderCoefTiDefault)
    sliderCoefTd.set(sliderCoefTdDefault) 

"""Plotting the graph, here the hold on function form matlab is tried to achieved by updating the points in the same graph."""
def PlotGraph():
    global x_pos
    global setpoint
    global time_start_graph
    global fig, canvas, a, line1, line2, mem, x_value, time_start, mem, mem_changed, exiting

     
    while(exiting==False):   
        if len(x_value) >1:
            if x_value[len(x_value)-1] > x_value[0]+mem:
                indices = [i for i in range(len(x_value)) if x_value[len(x_value)-1]-mem>x_value[i]]
                
                if len(indices)>0:                
                    for i in sorted(indices, reverse=True):
                        del x_value[i]
                        del y_value[i]
                        del y_setpointvalue[i]
                        
                    time_start=time_start+x_value[0]                    
                    x_value=[round(x - x_value[0],2) for x in x_value]
        else:
            time_start=time.time()                    
                          
                                            
        x_value.append(round(time.time()-time_start,2))
        y_value.append(x_pos)
        y_setpointvalue.append(setpoint)
         
    
    
        if not mem_changed == 0:
                a.set_xlim(0, mem)
                a.set_ylim(0, tube_length)
                line1,=a.plot([],[],'b')
                line2,=a.plot([],[],'r')  
                canvas.draw()
                canvas.flush_events()    
                background=fig.canvas.copy_from_bbox(a.bbox)                
                mem_changed=0
    
        time.sleep(0.03)
       
    PlotGraph()
                
time_start_graph=0  
def plotLines():
    global runningScope, line1, line2, x_value, y_value, y_setpointvalue, time_start_graph, exciting
    while(exiting==False): 
        if runningScope== True:       
    
            if time_start_graph==0:
                time_start_graph=time.time()
                Nr_fr=0
            
            if time.time()-time_start_graph>2:
                #print(Nr_fr/2)
                Nr_fr=0
                time_start_graph=time.time()              
            try:
                Nr_fr=Nr_fr+1
                fig.canvas.restore_region(background)
                line1.set_data(x_value,y_setpointvalue)
                line2.set_data(x_value,y_value)                
                
                
                a.draw_artist(line1)
                a.draw_artist(line2)
                
                
                canvas.blit()
                canvas.flush_events()   
            except:
                pass
            
        time.sleep(0.001)
    
        


"""Function to manage the variables which has to be saved in configuration file."""
def datatofile():
    global camera_no
    global FPS
    global tube_length
    global x_borderMin
    global x_borderMax
    global lowerBound_ball_input
    global upperBound_ball_input
    global neutral_angle
    global min_PWM
    global max_PWM
    global time_signaltofans
    global baudrate
    global com_port
    global deadband_error
    global deadband_angle
    global l_h ,l_s,l_v ,u_h,u_s,u_v
    global initial_max_PWM
    global initial_min_PWM
    global mem, runningScope
    global I_switch, D_switch, LPF_bool, D_kick_bool, openloop_switch_bool
    
    lowerBound_ball_input = np.array ([l_h,l_s,l_v])
    upperBound_ball_input = np.array([u_h,u_s,u_v])

    
    config['Camera'] = {'FPS':FPS,
                        'Camera_no': camera_no,
                        'Platforms_length': tube_length,
                        'pix_minpos': x_borderMin,
                        'pix_maxpos': x_borderMax,
                        'HSV_upperbound': upperBound_ball_input,
                        'HSV_lowerbound': lowerBound_ball_input,
                        'l_h':l_h ,                            
                        'l_s': l_s,
                        'l_v':l_v ,
                        'u_h':u_h  ,                            
                        'u_s': u_s,
                        'u_v': u_v
                         }

    config['System'] = {'neutral angle': 0,
                        'minimum_PWM': sliderminPWM.get(),
                        'maximum_PWM': slidermaxPWM.get(),
                        'deadtime_fans': time_signaltofans,
                        'com_port': com_port,
                        'baudrate': baudrate
                         }
    
    config['PID'] = {'deadband_error': deadband_error,

                        'K': sliderCoefK.get(),
                        'Ti': sliderCoefTi.get(),
                        'Td': sliderCoefTd.get(),
                        'deadband_angle': 0,
                        'Tf_factor':sliderCoefTf_factor.get(),
                        'I_active': I_switch,
                        'D_active': D_switch,
                        'D_kick_active': D_kick_bool,
                        'LPF_filter': LPF_bool,
                        'open_loop': openloop_switch_bool,
                        'manipulator': manipulator_list.get()
                         }
    
    config['Scope'] = {'mem': mem,
                        'runningScope': runningScope
                         }
    

"""Function to save the config file"""    
def save():
    with open('config.ini', 'w') as configfile:
        config.write(configfile)
                
def resetIntegrator():
    global integ, prev_integ
    integ=0
    prev_integ = 0
        
"""Function where the PID controller algorithm is implemented"""
error_int=0
def PID_Controller(ball_pos,set_pos):    
    global previous_error, error_int
    global timenow    
    global integ
    global prev_elapsedTime
    global prev_integ
    global prev_dreriv
    global pid_i
    global I_switch
    global D_switch
    global saturation
    global I_switch_bool
    global prev_PID
    global openloop_switch
    global time_datatoservo
    global time_save
    global PID
    global servo_signal
    global error
    global prev_ball_pos
    global LPF_bool
    global filename
    global start_rec_bool
    global tf_factor
    global min_PWM
    global max_PWM
    global neutral_angle
    global time_signaltofans
    global deadband_angle
    global deadband_error
    global deadband_angle_switch_bool
    global deadband_error_switch_bool
    global D_switch_bool
    global LPF_bool   
    global D_kick_bool
    global deriv
    global config
    global elapsedTime

    
    """Reading the parameters from the sliders"""
    K = sliderCoefK.get()
    Ti = sliderCoefTi.get()
    Td = sliderCoefTd.get()
    tf_factor = sliderCoefTf_factor.get()
    Tf = tf_factor * Td
    
    deadband_angle = 0
    deadband_error = sliderCoefdeadband_error.get()
    max_PWM = int(slidermaxPWM.get())
    min_PWM = int(sliderminPWM.get())
    

#PID calcuation
    
    error = set_pos - round(float((ball_pos * openloop_switch)),2)
    
    error_int=error_int+error
    
    lbl_E.config(text=str(round(error,2))) 
   
    if abs(error)<=deadband_error:
        error=0
    else:
        pass    
    
    lbl_E2.config(text=str(round(error,2))) 
    lbl_POS.config(text=str(round(float((ball_pos * openloop_switch)),2))) 
      
    time_previous=timenow    
    timenow = time.time()    
    elapsedTime = timenow - time_previous
    elapsedTime=min(elapsedTime,1)
    # computing Proportional controller
    Prop = K * error
    
    
    # computing Integral controller, variable I_switch determines if the branch is turned on. 
    integ = ((elapsedTime * error) + prev_integ) * I_switch
        

    if D_switch_bool == True or D_kick_bool == True:
        
        if D_kick_bool == True and LPF_bool == True:
            """ Algorithm derivative on PV with low pass filter"""
            deriv = (((2*K * Td* (-ball_pos+prev_ball_pos)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))#*D_switch
            # print("D on PV with LPF")
   
        elif D_switch_bool == True and LPF_bool == True:
            """ Algorithm derivative on error with LPF"""
            deriv = (((2*K * Td* (error-previous_error)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))*D_switch
            # print("D on error with LPF")
            
        elif D_kick_bool == True:
            """ Ideal derivative on PV"""
            deriv = (((2*K*Td*(-ball_pos+prev_ball_pos)/elapsedTime) - prev_deriv))*D_switch
            # print("Ideal D on PV")
            
        else:
            """Ideal derivative on error"""
            deriv = (((2*K*Td*(error-previous_error)/elapsedTime) - prev_deriv))*D_switch
            #print("ideal D on error")
    
    if abs(error)<=deadband_error and abs(previous_error)<=deadband_error:
        deriv=0
        integ=0

    PID =  Prop + (K/Ti)*integ + deriv
       
    lbl_KE.config(text=str(round(Prop,2)))
    lbl_KE2.config(text=str(round(Prop,2)))
    lbl_KI.config(text=str(round((K/Ti)*integ,2)))
    lbl_KD.config(text=str(round(deriv,2)))
    lbl_C.config(text=str(round(PID,2)))
    

    # Checking the selection for the signal manipulation and computing accordingly
    selection = manipulator_list.get()
    if selection == "O": 
        PID = (PID + min_PWM)
    if selection == "1": 
        PID = PID
    if selection == "Σ":
       PID = (prev_PID + PID)
    if selection == "Δ":
        PID =  (PID - prev_PID)
        
    
    
    
    if abs(PID) < deadband_angle :
        PID = 0
    else:
        pass
   

    lbl_MV.config(text=str(round(PID,2)))


    if PID<=min_PWM:
        PID=min_PWM
    if PID>=max_PWM:
         PID=max_PWM   
    PID = round(PID)
    
    servo_signal = PID
    servo_signal_beforelimiting = servo_signal # to check the saturation
    
    
    lbl_MVL.config(text=str(round(servo_signal)))
    lbl_MVL2.config(text=str(prev_PID))
 
    time_save = round(time.time() - time_start,2)
    row_contents = [time_save,setpoint,(round(error,2)),PID,servo_signal,round(x_incm,2)]
    # Append a list as new line to an old csv file
    if start_rec_bool == True:
        append_list_as_row(filename, row_contents)
        
    
    previous_error = error
    prev_elapsedTime = elapsedTime
    prev_integ = integ
    prev_deriv = deriv
    prev_PID = PID
    prev_ball_pos = ball_pos
    servo_signal=round(servo_signal)

    neutral_angle = 0

    try:
        ser.write(str(chr(round(servo_signal))).encode())
        ser.flush()
        
    except:
        tk.messagebox.showerror("showerror","Error while sending a signal to arduino.")
        arduino_connected_bool=False
        Start_PID=False
        startPID()


"""
Following function is adapted from the website:        
OpenCV Track Object Movement-PyImageSearch. “Ball Tracking with OpenCV.” PyImageSearch, 14 Sept. 2015,
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/.
Accessed 14 October 2020
"""
def show_frame():
    global showVideoWindow
    if exiting==False:
        if showVideoWindow == True:
            #width = int(img.shape[1] * scale_percent / 100)
            #height = int(img.shape[0] * scale_percent / 100)
            dim = (200, 200)
            # dim = (100000, 100000)
  
            # resize image
            #imgp = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
            imgp=cv2.resize(img,None,fx=0.9*900/camWidth,fy=0.9*500/camHeight)

            imgp = cv2.cvtColor(imgp, cv2.COLOR_BGR2RGB)
            imgp = Image.fromarray(imgp)
    
            imgtk = ImageTk.PhotoImage(image=imgp)
            lmain.imgtk = imgtk
            lmain.configure(image=imgtk)    
            lmain.after(20,show_frame)
        else:
            lmain.after(200,show_frame)

videoalreadyshown=False    
def read_cam():
    global camera_connected_bool, lowerBound_ball, upperBound_ball
    global tube_length, x_borderMin, x_borderMax, ballpos_tocalibrate, x_incm, x_pos
    global start_timeFPS, img, actual_fps, cam, nrFrame, videoalreadyshown, cnts_ball
        
    while(exiting==False): 
        if camera_connected_bool == True:    
            
            
            if camera_connected_bool == True:        
                if start_timeFPS==0:
                    start_timeFPS = time.time()
                    nrFrame=0
            
                _, img=cam.read()
                # to be tested
                img=cv2.flip(img,1)
                #img = img[0:int(camHeight),int((camWidth-camHeight)/2):int(camWidth-((camWidth-camHeight)/2))] 
                try:
                    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
                    
                    lowerBound_ball=np.array(lowerBound_ball_input)
                    upperBound_ball=np.array(upperBound_ball_input)
                    
                    mask_ball = cv2.inRange(imgHSV,lowerBound_ball,upperBound_ball)
                    mask_ball = cv2.blur(mask_ball,(6,6))                        
                    mask_ball = cv2.erode(mask_ball, None, iterations=2)         
                    mask_ball = cv2.dilate(mask_ball, None, iterations=2)
                
                    cnts_ball = cv2.findContours(mask_ball.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    cnts_ball = imutils.grab_contours(cnts_ball)
                    
            
        
        
        
                    if len(cnts_ball) > 0 :#and len(cnts_borderMax)>0 and len(cnts_borderMin)>0:
                        c_ball = max(cnts_ball, key=cv2.contourArea)
                        (x, y), radius = cv2.minEnclosingCircle(c_ball)
                        ballpos_tocalibrate = x
                        if x_borderMin > 0 and  x_borderMax> 0:
                            x_incm = round((tube_length/(x_borderMax-x_borderMin)*(x-x_borderMin)),1) 
                            x_pos = round(float(x_incm),1)
                            
                        time_save = round(time.time() - time_start,2)
                    # time,position,input,error
                        if openloop_switch_bool== False:
                            row_contents = [time_save,setpoint,setpoint,PID,servo_signal,round(x_incm,2)]
                    # Append a list as new line to an old csv file
                            if start_rec_bool == True:
                                append_list_as_row(filename, row_contents)
                        else:
                            row_contents = [time_save,setpoint,round(error,2),PID,servo_signal,round(x_incm,2)]
                    # Append a list as new line to an old csv file
                            if start_rec_bool == True:
                                append_list_as_row(filename, row_contents)    
                        if radius > 10:
                            if x_pos >0:                    
                            # cv2.putText(img,str(int(x_incm)) + ";" + str(int(y)).format(0, 0),(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                                cv2.putText(img,str(round((x_pos),1)) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                                cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)                
                            else:                
                                cv2.putText(img,str(round((x),1)) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                                cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)    
                except:
                    pass
                nrFrame=nrFrame+1            
                
            if (time.time() - start_timeFPS)>1:                 
                actual_fps = round(nrFrame / (time.time() - start_timeFPS),2)
                nrFrame=0
                start_timeFPS=0
            
            if showVideoWindow == True and videoalreadyshown==False:
                videoalreadyshown=True
                
                #imgp = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                #imgp = Image.fromarray(imgp)
    
                #imgtk = ImageTk.PhotoImage(image=imgp)
                #lmain.imgtk = imgtk
                #lmain.configure(image=imgtk)            
                lmain.after(20,show_frame)
            # to be tested
            #time.sleep(0.01)
        else:
            time.sleep(0.1)
    time.sleep(0.01)
                             
        


def main():
    global x 
    global x_pos
    global camWidth 
    global camHeight
    global timeInterval
    global start_time
    global showVideoWindow
    global time_start_measinfo
    global setpoint
    global tube_length
    global x_incm
    global Start_PID
    global deadtime
    global PID
    global servo_signal
    global error
    global stepinput_bool
    global ballpos_tocalibrate
    global x_borderMin
    global x_borderMax
    global filename
    global start_rec_bool
    global cam
    global camera_connected_bool
    global lowerBound_ball_input
    global upperBound_ball_input
    global time_signaltofans
    global config
    global actual_fps
    global GraphThread_started, thread, img

    
    setpoint = slider_setpos.get()
    
    time_signaltofans = sliderdeadtime.get()
    
        

    if camera_connected_bool == True:
        lbl_camera_connect.config(text = "camera is connected")
                      

        if time.time() - time_start_measinfo >=1 :        
            time_start_measinfo = time.time()                              
            lbl_PID_gegevens.config(text="FPS = "+str(actual_fps) )

    
    
    if Start_PID == True:
        if time.time()- deadtime > time_signaltofans: 
            PID_Controller(x_incm, setpoint)
            deadtime = time.time()

    lmain.after(5, main)


                  

"""The following function is retrieved from:
Praveen. “How to Find HSV Range of an Object for Computer Vision Applications?” Programming_fever, 28 July 2020,
https://medium.com/programming-fever/how-to-find-hsv-range-of-an-object-for-computer-vision-applications-254a8eb039fc.

Accessed 18 October 2020

"""
def run_all(camera):
    global camera_connected_bool
    def nothing(x):
        pass
    global l_h ,l_s,l_v ,u_h,u_s,u_v

    # Initializing the webcam feed.
    # to be tested
    #cap = cv2.VideoCapture(camera,cv2.CAP_DSHOW)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH,camWidth)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,camHeight)
    
    
    # Create a window named trackbars.
    cv2.namedWindow("HSV calibration")
    
    # Now create 6 trackbars that will control the lower and upper range of 
    # H,S and V channels. The Arguments are like this: Name of trackbar, 
    # window name, range,callback function. For Hue the range is 0-179 and
    # for S,V its 0-255.
    cv2.createTrackbar("L - H", "HSV calibration", 0, 179, nothing)
    cv2.createTrackbar("L - S", "HSV calibration", 0, 255, nothing)
    cv2.createTrackbar("L - V", "HSV calibration", 0, 255, nothing)
    cv2.createTrackbar("U - H", "HSV calibration", 179, 179, nothing)
    cv2.createTrackbar("U - S", "HSV calibration", 255, 255, nothing)
    cv2.createTrackbar("U - V", "HSV calibration", 255, 255, nothing)
     
    cv2.setTrackbarPos("L - H", "HSV calibration",l_h)
    cv2.setTrackbarPos("L - S", "HSV calibration",l_s)
    cv2.setTrackbarPos("L - V", "HSV calibration",l_v)
    cv2.setTrackbarPos("U - H", "HSV calibration",u_h)
    cv2.setTrackbarPos("U - S", "HSV calibration",u_s)
    cv2.setTrackbarPos("U - V", "HSV calibration",u_v)


    
    while True:
        
        # Start reading the webcam feed frame by frame.
        ret, frame = cam.read()
        
        if not ret:
            break
        # Flip the frame horizontally (Not required)
        # to be tested
        frame=cv2.flip(frame, 1)
        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get the new values of the trackbar in real time as the user changes 
        # them
        l_h = cv2.getTrackbarPos("L - H", "HSV calibration")
        l_s = cv2.getTrackbarPos("L - S", "HSV calibration")
        l_v = cv2.getTrackbarPos("L - V", "HSV calibration")
        u_h = cv2.getTrackbarPos("U - H", "HSV calibration")
        u_s = cv2.getTrackbarPos("U - S", "HSV calibration")
        u_v = cv2.getTrackbarPos("U - V", "HSV calibration")
     
        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        
        # Filter the image and get the binary mask, where white represents 
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)
     
        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Converting the binary mask to 3 channel image, this is just so 
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3,frame,res))
        
        # Show this stacked frame at 40% of the size.
        
        cv2.imshow("HSV calibration",cv2.resize(stacked,None,fx=0.9*600/camWidth,fy=0.9*500/camHeight))
        
        # If the user presses ESC then exit the program
        key = cv2.waitKey(1)
        if key == 27:
            break
        if cv2.getWindowProperty("HSV calibration", cv2.WND_PROP_VISIBLE) <1:
            config['Camera'] = {                
                                'l_h':l_h ,                            
                                'l_s': l_s,
                                'l_v':l_v ,
                                'u_h':u_h  ,                            
                                'u_s': u_s,
                                'u_v': u_v                                
                         }
            save()
            break
    #cap.release()
    cv2.destroyAllWindows()

def exit_function():
    global ser, cam, camera_connected_bool, exiting
    
    try:
        exiting=True
        camera_connected_bool=False
        time.sleep(1)
        cam.release()
        cv2.destroyAllWindows()   
    except:
        pass

    try:
        ser.close()
    except: 
        pass

    
    try:        
        main_Window.quit()    
        main_Window.destroy()        
    except:
        pass

def closevideo():
    global showVideoWindow
    videoWindow.withdraw()
    showVideoWindow = False
    Btn_ShowVideo["text"] = "Show video"
    
main_Window.protocol('WM_DELETE_WINDOW', exit_function)
videoWindow.protocol("WM_DELETE_WINDOW",closevideo)
systemcalibration_window.protocol("WM_DELETE_WINDOW",showsystemcalibration_window)
cameracalibration_window.protocol("WM_DELETE_WINDOW",showcameracalibration_window)

fig = Figure(figsize=(DPI_ratio*7.5, DPI_ratio*5.5))          
a = fig.add_subplot(111)
fig.tight_layout()
a.grid(True)
line1,=a.plot([],[],'b')
line2,=a.plot([],[],'r')  
a.set_xlim(0, mem)
a.set_ylim(0, tube_length)




class CustomNavigationToolbar(NavigationToolbar2Tk):
    def __init__(self, canvas, parent):
        super().__init__(canvas, parent)

    def save_figure(self, *args):
        global mem_changed
        global runningScope 
        # Override the save_figure method here                
        runningScope=0
        runningScopeWindow_plot()
        super().save_figure()



a.legend(["SP",'PV'], loc='upper right',fancybox=True, shadow=False, ncol=1, facecolor='white',frameon=True,framealpha=1,fontsize="16")
canvas = FigureCanvasTkAgg(fig, main_Window)
canvas.get_tk_widget().place(x=screenwidthratio*1120, y=screenwidthratio*190)
toolbar = CustomNavigationToolbar(canvas, main_Window)
toolbar.update()
toolbar.place(x=screenwidthratio*1040, y=screenwidthratio*620)
canvas.draw()
canvas.flush_events()    
background=fig.canvas.copy_from_bbox(a.bbox)



main()

main_Window.lift()



if os.path.isfile("config.ini"):
    msg_box = tk.messagebox.askquestion('Load previous configuration', 'Config file has been found. Do you wish to load it (autoconnect)?')
    if msg_box == 'yes':
        autoconnectfun()



mem_changed=1

thread1=Thread(target=PlotGraph)
thread1.setDaemon(True)
thread1.start()

thread2=Thread(target=plotLines)
thread2.setDaemon(True)
thread2.start()


thread3=Thread(target=read_cam)        
thread3.setDaemon(True)
thread3.start()




tk.mainloop()

