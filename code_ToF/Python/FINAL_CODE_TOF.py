"""
Created on Tue Mar 30 17:46:07 2021
Modified in 2023,2024 by Péter Zoltàn Csurcsia 
Modified in 2024 by Nellie Van Eeckhaute & Cloë Theys
@author: Pujan Bhandari
"""



""" Used libraries """
from tkinter import tix
import tkinter as tk
import numpy as np
import time 
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib import backend_bases
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import sys
import os
import glob
import serial
import re
import struct
import threading



# setting runtime configuration settings for matplotlib
plt.rcParams['savefig.dpi'] = 500
 #:'x-large'
plt.rcParams['ytick.labelsize']= 16
plt.rcParams['legend.fontsize']= 16
plt.rcParams['xtick.labelsize']= 16
plt.rcParams['font.size']= 10

backend_bases.NavigationToolbar2.toolitems = (
        ('Home', 'Reset original view', 'home', 'home'),
        ('Zoom', 'Zoom to rectangle', 'zoom_to_rect', 'zoom'),    
      )



""" Used global variables """
global I_switch # variable to check if the integral branch is turned on
global D_switch # variable to check if the derivative branch is turned on
global setpoint # variable to store the reference value
global tube_length # variable to store the length of the tube
global PV # variable to store the position of the ball in cm

global previous_time
global prev_elapsedTime 
global prev_integ
global prev_deriv
global prev_PID
global openloop_switch # variable to check if we're working in an open or closed loop
global deadtime # variable for the deadtime of the fans, the interval to send the signal to the serial path 
global PID # the pwm value we're sending to the fans, calculated by the PID controller
global com_port # name of the com_port where the arduino is connected
global error # variable for error signal
global prev_PV # previous position of the ball in cm
global LPF_bool # variable to check if the low pass filter is used
global D_kick_bool # variable to check if the bypass path to avoid derivative kick is activated
global SPS # amount of samples per second of the sensor

global x_incm_abs # helping variable to denote the position of the ball in cm for the minimum and maximum possible height of the tube

global max_pos # maximum position where the ball can reach in cm
global min_pos # minimum position where the ball can reach in cm
global tf_factor # a factor which is used to multiply with the derivative time constant and use as the time constat for the low pass filter
global deriv 
global integ
global K, Ti, Td
global arduino_connected_bool # helping variable to give feedback if arduino is connected
global actual_SPS # variable to calculate the actual SPS of the camera during operation

global min_PWM
global max_PWM

global deadband_error # variable to store the value of the deadband for the error signal obtained from the slider in GUI

global baudrate # variable to store the baudrate for serial communication
global comports # variable to store all available comports 

# global config

global scope_mem # scope memory length in seconds
global scope_mem_changed # indiciate if the scope memory has been changed
global GraphThread_started # check if the scope thread has started
global line_setpoint, line_ballpos # plotting lines
global start_timeSPS, nrFrame # variables to calculate number of samples per second of sensor
global exiting # variable to know when program is exiting or still running

global ser # variable for the connected serial port



""" function to get absolute path to resource, works for dev and PyInstaller """
""" this function was made with the idea to provide users of the program with an .exe file so that they would only need to run this file in order for everything to work"""
def resource_path(relative_path):
    try:
        # pyinstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")
        
    return os.path.join(base_path, relative_path)



""" initializing the variables"""
data_lock = threading.Lock()

previous_time=0.0
scope_mem=60
scope_mem_changed=0
start_timeSPS=0
nrFrame=0
exiting=False
K=0
Ti=0.01
tf_factor=0
Td=0
GraphThread_started=False
previous_error=0.0
prev_elapsedTime = 0.0
timenow=0.0
integ =0.0
deriv = 0.0
prev_integ = 0.0
prev_deriv = 0.0
integer_max = 0.0
PID = 0.0
prev_PID = 0.0

error = 0.0
prev_PV = 0
tf_factor = 0
actual_SPS = 0 # this will be used to see the effective SPS of the camera during operation
deadtime = 1 # default value for the deadtime of of the servomotor, this is adjustable in the GUI

deadband_error = 0 # default value for the deadband on the error signal.

I_switch = 0
D_switch = 0
openloop_switch = 1

max_pos = 45
min_pos = 10

tube_length = round(max_pos-min_pos,2)
setpoint = tube_length/2

PV = 0.0
x_incm_abs = -0.0001
time_start  = time.time()

time_start_measinfo = time.time()
deadtime = time.time()

x_value = []
y_value = []
y_errorvalue = []
y_setpointvalue = []

min_PWM = 0
max_PWM = 255

baudrate = 115200 # this value must match the baudrate that arduino uses for the serial communication
com_port =''



""" the main window of GUI"""
main_Window = tix.Tk() # tix is used because it allows placing information as tooltip for various objects in GUI

screenwidth = int(np.minimum(main_Window.winfo_screenwidth()-10,1900*1.5))
screenwidthratio=screenwidth/1900

buttondrift=4/0.23157894736842158*(1-screenwidthratio)
if screenwidthratio>1.1:
    buttondrift=0.8/0.23157894736842158*(1-screenwidthratio)

main_Window.title("Ping Pong Ball: PID controller") # Title of the main window
main_Window.geometry(str(screenwidth) +"x"+str(int(screenwidthratio*800))+"+0-50") # size of the main window for GUI starting from 5 pixels right and under the topleft corner of the screen
main_Window["bg"]="light blue" # background colour of the mainwindow, not visible in GUI because of background image
main_Window.resizable(0, 0) # the GUI is not resizable because of the placement of different objects in the GUI

main_Window.call("tk","scaling",screenwidthratio)

DPI_in_use=main_Window.winfo_fpixels('1i') # dots per inch, resolution of the screen
DPI_ratio=108/DPI_in_use



"""importing the background image (blockdiagram), resizing it to make it fit in the main window of GUI,
placing a label on the main window and placing the resized image in the label"""
image_path_background = resource_path("Background_TOF_V1.png")
my_pic = Image.open(image_path_background) 
resized = my_pic.resize((screenwidth,int(screenwidthratio*800)),Image.Resampling.LANCZOS)
bg = ImageTk.PhotoImage(resized,master = main_Window)
bg_lbl = tk.Label(main_Window,image = bg)
bg_lbl.place(x= 0,y = 0, relwidth = 1, relheight = 1)



""" Window to calibrate the fans and connect to the arduino """
systemcalibration_window = tix.Toplevel(main_Window)
systemcalibration_window.geometry(str(int(screenwidthratio*650))+"x"+str(int(screenwidthratio*450)))

systemcalibration_window.title("System calibration")
systemcalibration_window["bg"]="white"
systemcalibration_window.resizable(0, 0) 
systemcalibration_window.withdraw()



""" Importing and resizing the images (switches) which will be placed to indicate if a certain branch is turned on or off"""
image_path_openH = resource_path("open_horizontal.png")
bg_open_horizontal = Image.open(image_path_openH)
bg_open_horizontal = bg_open_horizontal.resize((int(screenwidthratio*50),int(screenwidthratio*25)),Image.Resampling.LANCZOS)
bg_open_horizontal = ImageTk.PhotoImage(bg_open_horizontal, master = main_Window)

image_path_closedH = resource_path("closed_horizontal.png")
bg_closed_horizontal = Image.open(image_path_closedH)
bg_closed_horizontal = bg_closed_horizontal.resize((int(screenwidthratio*50),int(screenwidthratio*25)),Image.Resampling.LANCZOS)
bg_closed_horizontal = ImageTk.PhotoImage(bg_closed_horizontal, master = main_Window)

image_path_openV = resource_path("open_vertical.png")
bg_open_vertical = Image.open(image_path_openV)
bg_open_vertical = bg_open_vertical.resize((int(screenwidthratio*25),int(screenwidthratio*50)),Image.Resampling.LANCZOS)
bg_open_vertical = ImageTk.PhotoImage(bg_open_vertical, master = main_Window)

image_path_closedV = resource_path("closed_vertical.png")
bg_closed_vertical = Image.open(image_path_closedV)
bg_closed_vertical = bg_closed_vertical.resize((int(screenwidthratio*25),int(screenwidthratio*50)),Image.Resampling.LANCZOS)
bg_closed_vertical = ImageTk.PhotoImage(bg_closed_vertical, master = main_Window)

lbl_opencircuit_LPF= tk.Label(main_Window,image = bg_closed_horizontal,bg = "lightgreen") # we're default working with LPF turned off
lbl_opencircuit_LPF.place(x=screenwidthratio*950, y=screenwidthratio*363)



""" Setting default values for the sliders in the GUI"""
setposDefalult = 0
sliderCoefKDefault = 0 
sliderCoefTiDefault = 0.001 # otherwise division by zero in line 1357
sliderCoefTdDefault = 0 
sliderCoefTf_Default = 0 
slider_deadband_Default = deadband_error



""" Placing different objects in the main window of the GUI at suitable places accordingly on the block diagram. 
Firstly all the objects are placed in the main window, than the objects for system calibration are placed"""
# Sliders
slider_setpos = tk.Scale(main_Window, from_=0, to=tube_length, orient="horizontal",bg = "white", length=screenwidthratio*150, width = screenwidthratio*15, resolution=0.1)
slider_setpos.set(setposDefalult)
slider_setpos.place(x=screenwidthratio*3,y = screenwidthratio*170)

sliderCoefK = tk.Scale(main_Window, from_=0, to=10, orient="horizontal",bg = "white", length=screenwidthratio*140, width = screenwidthratio*15, resolution=0.001)
sliderCoefK.set(sliderCoefKDefault)
sliderCoefK.place(x=screenwidthratio*380,y = screenwidthratio*270)

sliderCoefTi = tk.Scale(main_Window, from_=0, to=5, orient="horizontal",bg = "white", length=screenwidthratio*200, width = screenwidthratio*15, resolution=0.001)
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
sliderminPWM.set(0)
sliderminPWM.place(x =screenwidthratio*1310,y=screenwidthratio*45)


# Buttons
Btn_StartPID = tk.Button(main_Window, text = "Start controller ",bg = 'lightgreen', command=lambda:startPID())
Btn_StartPID.place(x=screenwidthratio*5, y=screenwidthratio*240)

Btn_reset = tk.Button(main_Window, text = "Reset controller",bg = 'white', command=lambda:reset_fun())
Btn_reset.place(x=screenwidthratio*5, y=screenwidthratio*290)

Btn_Switch_I = tk.Button(main_Window, image = bg_open_horizontal, bg= "red", command = lambda:ISwitch())
Btn_Switch_I.place(x=screenwidthratio*550, y=screenwidthratio*218)

Btn_Switch_D = tk.Button(main_Window,image = bg_open_horizontal,bg= "red", command = lambda: DSwitch())
Btn_Switch_D.place(x=screenwidthratio*550, y=screenwidthratio*363)

Btn_Switch_Derivkick = tk.Button(main_Window,image = bg_open_vertical,bg= "red", command = lambda:d_kickfun() )
Btn_Switch_Derivkick.place(x=screenwidthratio*627-buttondrift, y=screenwidthratio*400)

Btn_Switch_LPF = tk.Button(main_Window, image = bg_open_vertical,bg= "red", command = lambda:LPF_fun() )
Btn_Switch_LPF.place(x=screenwidthratio*868-buttondrift, y=screenwidthratio*388)

Btn_Switch_closedloop = tk.Button(main_Window, image = bg_closed_vertical,bg= "lightgreen", command = lambda: openloop_switchfun()) # by default we're working in a closed-loop system
Btn_Switch_closedloop.place(x=screenwidthratio*163-buttondrift, y=screenwidthratio*395)

Btn_systemcalibration_window = tk.Button(main_Window, text = "setup",bg = 'white', command=lambda:showsystemcalibration_window())
Btn_systemcalibration_window.place(x=screenwidthratio*1520, y=screenwidthratio*137)

Btn_ResetIntergrator = tk.Button(main_Window, text = "R", command=lambda:resetIntegrator())
Btn_ResetIntergrator.place(x=screenwidthratio*820, y=screenwidthratio*220)
tip_Btn_ResetIntergrator = tix.Balloon(main_Window)
tip_Btn_ResetIntergrator.bind_widget(Btn_ResetIntergrator, balloonmsg = "Reset the integrator.") 


# Label
lbl_sps = tk.Label(main_Window,text="0 SPS",bg = 'white',fg = 'black')
lbl_sps.place(x=screenwidthratio*830, y=screenwidthratio*750)



""" Frame for the scope, where buttons are packed to control the plot"""
FrameScope = tk.LabelFrame(main_Window, text="Scope", bg ='White')
FrameScope.place(x=screenwidthratio*970,y=screenwidthratio*550,width=screenwidthratio*190/DPI_ratio)

Btn_graph = tk.Button(FrameScope, text="Run",bg = 'white', relief="sunken", command=lambda:runningScopeWindow())
Btn_graph.grid(column = 0, row = 0, sticky = tk.W, padx=screenwidthratio*5)

lbl_scope_mem = tk.Label(FrameScope,text="(s)",bg = 'white',fg = 'black')
lbl_scope_mem.grid(column=2, row=0, sticky = tk.W)
txtinput_scope_mem = tk.Entry(FrameScope, width=int(4*screenwidthratio))
txtinput_scope_mem.insert(0, scope_mem)
txtinput_scope_mem.grid(column=1, row=0, sticky = tk.W)


def callback_scope_mem(sv): # function called when scope memory (time axis domain) has changed
    global scope_mem, scope_mem_changed    
    if len(txtinput_scope_mem.get())>0: 
        if scope_mem < int(txtinput_scope_mem.get()):
            scope_mem_changed=-1
        elif scope_mem > int(txtinput_scope_mem.get()):
            scope_mem_changed=1            
        scope_mem = int(txtinput_scope_mem.get())        
    else:
        scope_mem=10
    # constraining scope memory to [10s,10min]
    if scope_mem<10:
        scope_mem=10
        tk.messagebox.showwarning("Warning", "Minimum memory length is 10 seconds")
    elif scope_mem>600:
        scope_mem=600
        tk.messagebox.showwarning("Warning", "Maximum maximum length is 10 minutes")


# keybindings
txtinput_scope_mem.bind('<Return>', callback_scope_mem)
txtinput_scope_mem.bind('<FocusOut>', callback_scope_mem)



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


def callback_setpos(event): # function called when setpoint is changed through the slider
    global setpoint
    setpoint = slider_setpos.get()


# keybindings
slider_setpos.bind('<Motion>', callback_setpos)



""" Frame for indication of connected Arduino and busy controller, the background colour changes accorindgly"""
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



"""labels used """
lbl_E = tk.Label(main_Window,text="E",bg = 'white',fg = 'black') # error before deadband
lbl_E.place(x=screenwidthratio*205, y=screenwidthratio*200)

lbl_E2 = tk.Label(main_Window,text="E",bg = 'white',fg = 'black') # error after deadband
lbl_E2.place(x=screenwidthratio*360, y=screenwidthratio*200)

lbl_POS = tk.Label(main_Window,text="PV",bg = 'white',fg = 'black') # position variable, current position of the ball
lbl_POS.place(x=screenwidthratio*180, y=screenwidthratio*270)

lbl_KE = tk.Label(main_Window,text="KE",bg = 'white',fg = 'black')
lbl_KE.place(x=screenwidthratio*500, y=screenwidthratio*200)
lbl_KE2 = tk.Label(main_Window,text="KE",bg = 'white',fg = 'black')
lbl_KE2.place(x=screenwidthratio*1040, y=screenwidthratio*65)

lbl_KI = tk.Label(main_Window,text="KIE",bg = 'white',fg = 'black')
lbl_KI.place(x=screenwidthratio*1040, y=screenwidthratio*105)

lbl_KD = tk.Label(main_Window,text="KED",bg = 'white',fg = 'black')
lbl_KD.place(x=screenwidthratio*1120, y=screenwidthratio*165)

lbl_C = tk.Label(main_Window,text="C",bg = 'white',fg = 'black') # controller variable, combining P I and D branches
lbl_C.place(x=screenwidthratio*1140, y=screenwidthratio*100)

lbl_MV = tk.Label(main_Window,text="MV",bg = 'white',fg = 'black') # manipulated value
lbl_MV.place(x=screenwidthratio*1300, y=screenwidthratio*100)

lbl_MVL = tk.Label(main_Window,text="MV-S",bg = 'white',fg = 'black') # manipulated value after constraining to min and max PWM
lbl_MVL.place(x=screenwidthratio*1600, y=screenwidthratio*100)

lbl_MVL2 = tk.Label(main_Window,text="MV-D",bg = 'white',fg = 'black') # manipulated value after deadtime
lbl_MVL2.place(x=screenwidthratio*1750, y=screenwidthratio*100)



""" Variables to give a tool-tip for selected objects"""
tip_Iswitch = tix.Balloon(main_Window)
tip_Dswitch = tix.Balloon(main_Window)
tip_D_kick = tix.Balloon(main_Window)
tip_LPF = tix.Balloon(main_Window)
tip_openloop = tix.Balloon(main_Window)

tip_tf_factor = tix.Balloon(main_Window)
tip_tf_factor.bind_widget(sliderCoefTf_factor, balloonmsg = "Chosen value will be multiplied with the "+"\n"+"timeconstant (Td) of derivative controller.")

tip_minPWM = tix.Balloon(main_Window)
tip_minPWM.bind_widget(sliderminPWM, balloonmsg = "This is the minimum PWM")

tip_deadband_error = tix.Balloon(main_Window)
tip_deadband_error.bind_widget(sliderCoefdeadband_error, balloonmsg = "Within this deadband, the output of the controller"+"\n"+"will be null.")

tip_deadtime = tix.Balloon(main_Window)
tip_deadtime.bind_widget(sliderdeadtime, balloonmsg = "This is the time difference between "+"\n"+"two PWM signals.")

tip_maxPWM = tix.Balloon(main_Window)
tip_maxPWM.bind_widget(slidermaxPWM, balloonmsg = "This is the maximum PWM")



""""dropdown menu for selecting manipulating operation from the signal manipulator block"""
manipulator_list = tk.StringVar()
manipulator_list.set("1")
drop_block = tk.OptionMenu(main_Window, manipulator_list, "1", "\u0394", "\u03A3","O") # \u0394 : Unicode for Greek capital letter Delta, \u03A3 : Unicode for Greek capital letter Sigma
# I changed the delta and sigma signs to unicode representation, assuring that these symbols remain the same on all devices
drop_block.place(x=screenwidthratio*1210,y = screenwidthratio*120)



""" Placing different objects in the window for system calibration """ 
Btn_arduino_refresh= tk.Button(systemcalibration_window, text = "Refresh com ports", command = lambda: refresh_comports())
Btn_arduino_refresh.place(x = screenwidthratio*200, y = screenwidthratio*20)

lbl_arduino_connect = tk.Label(systemcalibration_window,text="Arduino not connected",bg = 'red',fg = 'black')
lbl_arduino_connect.place(x = screenwidthratio*200, y = screenwidthratio*60)

lbl_extra_info = tk.Label(systemcalibration_window,text="1. Connect your Arduino device."+"\n"+ "2. Experiment with the PWM values \n using the \"Test PWM value\" input on the right.\n Provide PWM values in the range [0,255]."+"\n\n"+"3. Change the PWM to first find the max and then the min position of the ball.\n At this position will be the min and max PWM values set (i.e., the red and blue slider)."+"\n"+"Later on you can readjust these values manually if it is needed.",bg = 'white',fg = 'black', justify=tk.LEFT)
lbl_extra_info.place(x= screenwidthratio*10,y = screenwidthratio*100)

lbl_PWM_to_fans = tk.Label(systemcalibration_window,text="Test PWM value",bg = 'white',fg = 'black')
lbl_PWM_to_fans.place(x= screenwidthratio*500,y = screenwidthratio*90)

PWM_checkangle = tk.StringVar(); PWM_checkangle.set("1")
txtinput_PWM_to_fans = tk.Entry(systemcalibration_window, width = int(screenwidthratio*3),textvariable=PWM_checkangle)
txtinput_PWM_to_fans.place(x= screenwidthratio*505,y = screenwidthratio*120)

btn_applyangle = tk.Button(systemcalibration_window,text = "Apply",command = lambda:signal_to_fans())
btn_applyangle.place(x = screenwidthratio*580,y = screenwidthratio*115)

Btn_arduino= tk.Button(systemcalibration_window, text = "Connect arduino", command = lambda: connect_arduinofun())
Btn_arduino.place(x = screenwidthratio*20, y = screenwidthratio*60)

Btn_applyall= tk.Button(systemcalibration_window, text = "Apply", command = lambda:applyall_fun())
Btn_applyall.place(x = screenwidthratio*60, y = screenwidthratio*410)

Btn_finish_systemcalibration = tk.Button(systemcalibration_window, text = "OK",bg = 'white', command=lambda:finish_systemcalibration())
Btn_finish_systemcalibration.place(x = screenwidthratio*20, y = screenwidthratio*410)


# balloons to pop up over a widget and provide users descriptive messages
tip_btn_applyangle = tix.Balloon(systemcalibration_window)
tip_btn_applyangle.bind_widget(btn_applyangle, balloonmsg = "Send given PWM value to the fan")


def callback_checkangle(sv):
    signal_to_fans()

def callback_stepinput(sv):
    step_inputfun()
    
def callback_update(event): # function to update present values in GUI
    global K, Ti, Td, tf_factor, Tf, deadband_error, max_PWM, min_PWM, deadtime, setpoint
   
    K = sliderCoefK.get()
    Ti = sliderCoefTi.get()
    Td = sliderCoefTd.get()
    tf_factor = sliderCoefTf_factor.get()
    Tf = tf_factor * Td
    
    setpoint = slider_setpos.get()
    deadband_error = sliderCoefdeadband_error.get()
    max_PWM = int(slidermaxPWM.get())
    min_PWM = int(sliderminPWM.get())
    
    deadtime = sliderdeadtime.get()
    

# keybindings: 
#    <Return> = enter 
#    <FocusOut> = keyboard focus moved from this widget to another widget
#    <ButtonRelease> = when button is released
txtinput_PWM_to_fans.bind('<Return>', callback_checkangle)
sliderCoefK.bind('<ButtonRelease>', callback_update)
slider_setpos.bind('<ButtonRelease>', callback_update)
sliderCoefTi.bind('<ButtonRelease>', callback_update)
sliderCoefTd.bind('<ButtonRelease>', callback_update)
sliderCoefTf_factor.bind('<ButtonRelease>', callback_update)
sliderminPWM.bind('<ButtonRelease>', callback_update)
sliderCoefdeadband_error.bind('<ButtonRelease>', callback_update)
slidermaxPWM.bind('<ButtonRelease>', callback_update)
txtinput_stepinput.bind('<Return>', callback_stepinput)


lbl_tube_length = tk.Label(systemcalibration_window,text="The useful height of the tube is",bg = 'white',fg = 'black')
lbl_tube_length.place(x=screenwidthratio*10,y=screenwidthratio*350)

lbl_tube_length_cm = tk.Label(systemcalibration_window,text="not yet determined",bg = 'white',fg = 'black')
lbl_tube_length_cm.place(x=screenwidthratio*240,y=screenwidthratio*350)

Btn_setminpos = tk.Button(systemcalibration_window, text = "set min position", command = lambda:set_minposfun())
Btn_setminpos.place(x =screenwidthratio* 180, y = screenwidthratio*280)

Btn_setmaxpos = tk.Button(systemcalibration_window, text = "set max position", command = lambda:set_maxposfun())
Btn_setmaxpos.place(x =screenwidthratio* 20, y =screenwidthratio* 280)

lbl_pos_min = tk.Label(systemcalibration_window,text="min position: not yet determined",bg = 'white',fg = 'black')
lbl_pos_min.place(x=screenwidthratio*330,y=screenwidthratio*273)

lbl_pos_max = tk.Label(systemcalibration_window,text="max position: not yet determined",bg = 'white',fg = 'black')
lbl_pos_max.place(x=screenwidthratio*330,y=screenwidthratio*298)



""" Here ends the construction of the GUI. The different functions used are constructed and assigned to the objects above. """
""" Function to read all the inputs from the system calibration window"""
def applyall_fun():
    global tube_length, scope_mem_changed, max_pos, min_pos, x_incm_abs
    try:
        if x_incm_abs<0:
            tk.messagebox.showwarning("Warning", "No position signal is obtained. Calibration failed.")
        else:            
            scope_mem_changed=1
            tube_length = round(max_pos-min_pos,2)
    except:
        tk.messagebox.showerror("Error","Error while entering calibration parameters") 
        
        
""" Function used for resetting the controller. All the sliders are set to their default values."""  
def reset_fun():
    global setpoint, PV
    global PID, integ, deriv, prev_PID, prev_integ, prev_deriv, error, previous_error
    global elapsedTime, prev_PV, prev_elapsedTime, scope_mem, scope_mem_changed, runningScope
    
    previous_error = 0
    prev_elapsedTime = 0
    prev_integ = 0
    prev_deriv = 0
    prev_PID = 0
    prev_PV = 0
    error = 0
    integ = 0
    deriv = 0
    scope_mem=60
    scope_mem_changed=1
    runningScope=1
    
    slider_setpos.set(PV)
    sliderCoefK.set(sliderCoefKDefault)
    sliderCoefTi.set(sliderCoefTiDefault)
    sliderCoefTd.set(sliderCoefTdDefault) 
    sliderCoefTf_factor.set(sliderCoefTf_Default)
    sliderCoefdeadband_error.set(slider_deadband_Default)
    txtinput_scope_mem.delete(0,tk.END)
    txtinput_scope_mem.insert(0, scope_mem)
    
    runningScopeWindow()
    
    try:
        ser.write(str(chr(int(0))).encode())
        ser.flush() # ensure data is transmitted
    except:
        pass


"""Function to finish system calibration and close the calibration window"""
def finish_systemcalibration():
    global show_systemcalibration_window
    global min_PWM
    global max_PWM
    global deadband_error
    global deadtime

    try:        
        applyall_fun()
        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "setup"
    except:
        tk.messagebox.showerror("showerror","Error while improting inputs") 
        
    

""" Function to refresh all the comports available and update the dropdown menu in the system calibration window.
The function is adapted from:
âSerial List Ports in Python - Serial_ports().â Gist,
https://gist.github.com/tekk/5c8d6824108dfb771a1e16aa8bc479f0.

Accessed 25 March 2021.

"""
def refresh_comports():
    global clicked_comport
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
        clicked_comport = tk.StringVar()
        clicked_comport.set("Choose a com port")
        drop_block_comport = tk.OptionMenu(systemcalibration_window, clicked_comport, *comports)
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
        clicked_comport = tk.StringVar()
        clicked_comport.set("Choose a com port")
        try:            
            drop_block_comport = tk.OptionMenu(systemcalibration_window, clicked_comport, *comports)
            drop_block_comport.place(x= screenwidthratio*20,y = screenwidthratio*20)
        except:
            lbl_arduino_connect.config(text = "No arduino found.")


"""Function to display calibration window for arduino and fan."""
show_systemcalibration_window = False
def showsystemcalibration_window():
    global show_systemcalibration_window
    global comports
    if show_systemcalibration_window == False:
        systemcalibration_window.deiconify()
        show_systemcalibration_window = True
        Btn_systemcalibration_window["text"] = "close"

    else:
        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "setup"
        

"""Function to display the graph in real time. The graph is updated each second. This can be improved"""
runningScope = True
def runningScopeWindow_plot():
    global runningScope
    if runningScope == True:
        Btn_graph.config(relief="sunken")        
    else:        
        Btn_graph.config(relief="raised")
        
        
def runningScopeWindow():
    global runningScope, scope_mem_changed, canvas
    if runningScope == False:
        runningScope = True
        canvas.draw()
        canvas.flush_events()   
        
    else:
        runningScope = False     
    runningScopeWindow_plot()

        
"""Function to read the selected com port and connect the arduino"""
arduino_connected_bool = False
def connect_arduinofun():
    global ser
    global arduino_connected_bool
    global baudrate
    global com_port
    global clicked_comport
    try:
        try:
            com_selection =str( clicked_comport.get())
            com_port = re.sub('\W', '', com_selection)             
        except:
            pass
        ser = serial.Serial(com_port,baudrate) 
        
        ser.write(str(chr(int(0))).encode())
        ser.flush() # ensure data is transmitted
                
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
    global Start_PID, ser
    if Start_PID == False:
        if arduino_connected_bool == False:
            tk.messagebox.showerror("PID controller cannot start","There is no arduino connected.")      
        else:            
            Start_PID = True
            Btn_StartPID["text"] = "Stop Controller"
            Btn_StartPID['bg'] = "red"
            lbl_indi_controller["bg"] = "lightgreen"
            Btn_StartPID.relief="sunken"
    else:
        Start_PID = False
        Btn_StartPID["text"] = "Start Controller"
        Btn_StartPID['bg'] = "lightgreen"
        lbl_indi_controller["bg"] = "red"
        Btn_StartPID.relief="raised"
        try:
            ser.write(str(chr(0)).encode())
            ser.flush() # ensure data is transmitted         
        except:
            pass   



"""The following functions are used to indicate if any particular branch is turned on, 
the boolean variable and the indicators are adjusted accordingly"""
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



"""Fuctions to connect and disconnect the feedback branch of the closed loop system"""
openloop_switch_bool = True
tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for an openloop system.")

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



D_switch_bool = False
tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 

def DSwitch_GUI():
    global D_switch_bool
    if D_switch_bool == True:
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn off the derivative.") 
        Btn_Switch_D["image"] = bg_closed_horizontal
        Btn_Switch_D['bg'] = "lightgreen"
        
        tip_D_kick.bind_widget(Btn_Switch_Derivkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Derivkick["image"] = bg_open_vertical
        Btn_Switch_Derivkick["bg"] = "red"
        
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
    
             
D_kick_bool = False
tip_D_kick.bind_widget(Btn_Switch_Derivkick, balloonmsg = "Click here to avoid derivative kick.")


def d_kickfun_GUI():
    global D_switch
    global D_switch_bool
    global D_kick_bool
    global tf_factor
    if D_kick_bool == True:
        tip_D_kick.bind_widget(Btn_Switch_Derivkick, balloonmsg = "Click here to disconnect this branch.")
        Btn_Switch_Derivkick["image"] = bg_closed_vertical
        Btn_Switch_Derivkick["bg"] = "lightgreen"

        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
        Btn_Switch_D["image"] = bg_open_horizontal
        Btn_Switch_D['bg'] = "red"
    else:
        tip_D_kick.bind_widget(Btn_Switch_Derivkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Derivkick["image"] = bg_open_vertical
        Btn_Switch_Derivkick["bg"] = "red"


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
    
    

"""Function to take stepsize from the user and set the new value of setpoint"""
def step_inputfun():
    global setpoint
    try:        
        setpoint = setpoint + float(txtinput_stepinput.get())
        slider_setpos.set(setpoint)        
    except:
        tk.messagebox.showerror("showerror","Error while taking inputs from user.")      


# CHECK THIS: deze zijn nog voor balancing beam, niet voor onze verticale buis!    
"""Following two functions are used to take the minimum and maximum PWM values and the corresponding ball height. These are used to calculate the useful tube length."""
def set_minposfun():
    global min_pos, max_pos
    global x_incm_abs
    global tube_length, scope_mem_changed
    try:                
        if x_incm_abs<0:
            tk.messagebox.showwarning("Warning", "No position signal is obtained.")
        else:
            scope_mem_changed=1
            sliderminPWM.set(int(txtinput_PWM_to_fans.get()))
            min_pos = x_incm_abs

            tube_length =round(max_pos-min_pos)
            lbl_tube_length_cm.config(text = str(tube_length)+" cm")
            slider_setpos["to"] = tube_length
                    
            lbl_pos_min.config(text = "min position: "+str(min_pos)+" cm @ "+txtinput_PWM_to_fans.get()+" PWM")
                    
            if min_pos>max_pos and max_pos>0:
                tk.messagebox.showwarning("Warning", "Your minimum position is greater than the maximum position")
        
    except:
        lbl_pos_min.config(text = "min position: error")


def set_maxposfun():
    global max_pos, min_pos
    global x_incm_abs
    global tube_length, scope_mem_changed
    
    try:
        if x_incm_abs<0:
            tk.messagebox.showwarning("Warning", "No position signal is obtained.")
        else:

            scope_mem_changed=1
            slidermaxPWM.set(int(txtinput_PWM_to_fans.get()))
            max_pos = x_incm_abs 
            tube_length =round(max_pos-min_pos,2)
            lbl_tube_length_cm.config(text = str(tube_length)+" cm")
            slider_setpos["to"] = tube_length
            lbl_pos_max.config(text = "max position: "+str(max_pos)+" cm @"+txtinput_PWM_to_fans.get()+" PWM")
            
            if max_pos<min_pos:
                tk.messagebox.showwarning("Warning", "Your maximum position is smaller than the minimum position")
    except:
        lbl_pos_max.config(text = "max position: error")
        
        
"""Function used to send a chosen PWM signal to the fan to get proper maximum and minimum PWM values"""
def signal_to_fans():
    global ser
    try:
        PWM_to_fans = int(txtinput_PWM_to_fans.get()) 
  
        ser.write(str(chr(int(PWM_to_fans))).encode())
        ser.flush() # ensure data is transmitted
        
    except:
        tk.messagebox.showerror("showerror","Error sending signal to arduino")       
    


"""Plotting the graph, here the hold on function form Matlab is tried to be achieved by updating the points in the same graph."""
def PlotGraph():
    global setpoint, PV
    global time_start_graph
    global fig, canvas, scope_figure, line_setpoint, line_ballpos, x_value, time_start, scope_mem, scope_mem_changed, exiting

     
    while(exiting==False):   
        if len(x_value) >1: # make running scope only display the last (default) 60 seconds
            if x_value[len(x_value)-1] > x_value[0]+scope_mem:
                indices = [i for i in range(len(x_value)) if x_value[len(x_value)-1]-scope_mem>x_value[i]]
                
                if len(indices)>0:                
                    for i in sorted(indices, reverse=True):
                        del x_value[i]
                        del y_value[i]
                        del y_setpointvalue[i]
                        
                    time_start=time_start+x_value[0]                    
                    x_value=[round(x - x_value[0],2) for x in x_value]
        else:
            time_start=time.time()                    
                          
        # add new values to plot arrays                                    
        x_value.append(round(time.time()-time_start,2))
        y_value.append(PV)
        y_setpointvalue.append(setpoint)
    
        if not scope_mem_changed == 0:
                scope_figure.set_xlim(0, scope_mem)
                scope_figure.set_ylim(-0.5, tube_length+3)
                line_setpoint,=scope_figure.plot([],[],'b')
                line_ballpos,=scope_figure.plot([],[],'r')  
                canvas.draw() # plot updated values
                canvas.flush_events()
                scope_mem_changed=0
    
        time.sleep(0.03)
       


def read_pos():
    global arduino_connected_bool, ser, min_pos, max_pos, PV,x_incm_abs, data_in
    global start_timeSPS, actual_SPS, nrFrame, x_incm_series
        
    while(exiting==False):
        if arduino_connected_bool == True:    
            value = read_position_from_sensor() # cm
            x_incm_abs = value
            
            PV=value-min_pos
            
            if start_timeSPS==0:
                start_timeSPS = time.time()
                nrFrame=0
            
            nrFrame=nrFrame+1            
                        
            if (time.time() - start_timeSPS)>1:                 
                actual_SPS = round(nrFrame / (time.time() - start_timeSPS),2)
                nrFrame=0
                start_timeSPS=0  
                lbl_sps.config(text =str(actual_SPS)+" SPS")
                
        else:
            x_incm_abs=-0.0001
            PV=0
                            
        time.sleep(0.001)


def read_integer_from_serial(): # function to read serial input from bytes to integer (ball position in mm)
    global ser    
   
    while ser.in_waiting < 2:  # Wait until there are enough bytes in the buffer, 2 bytes for an integer
        pass
    bytes_read = ser.read(2) # Read 2 bytes from the serial buffer
    received_number = struct.unpack('h', bytes_read)[0] # Unpack the bytes into an integer
    return received_number


def read_position_from_sensor():
    global ser
    value = read_integer_from_serial() # mm
    value = value/10.0 # cm
    return value

       
                
time_start_graph=0  
def plotLines():
    global runningScope, line_setpoint, line_ballpos, x_value, y_value, y_setpointvalue, time_start_graph, exciting, background
    while(exiting==False): 
        if runningScope== True:       
    
            if time_start_graph==0:
                time_start_graph=time.time()
                Nr_fr=0
            
            if time.time()-time_start_graph>2:
                Nr_fr=0
                time_start_graph=time.time()              
            try:
                Nr_fr=Nr_fr+1
                fig.canvas.restore_region(background)
                line_setpoint.set_data(x_value,y_setpointvalue)
                line_ballpos.set_data(x_value,y_value)                                
                
                scope_figure.draw_artist(line_setpoint) # faster rendering by using blit()
                scope_figure.draw_artist(line_ballpos)
                
                # blit() only redraws parts of the canvas that have changed or need to be updated
                # draw() refreshes the entire canvas by redrawing it, so better only used when needed
                                
                canvas.blit()
                canvas.flush_events()        
            except:
                pass
            
        time.sleep(0.001)
    
        
                
def resetIntegrator():
    global integ, prev_integ
    integ=0
    prev_integ = 0
        
"""Function where the PID controller algorithm is implemented"""
def PID_Controller():    
    global previous_error, timenow, prev_elapsedTime  
    global integ, prev_integ, prev_deriv, previous_time
    global I_switch, D_switch, I_switch_bool, prev_PID, openloop_switch
    global PID, error, PV, prev_PV, PF_bool, tf_factor
    global deadtime, deadband_error, deadband_error_switch_bool
    global D_switch_bool, LPF_bool, D_kick_bool
    global deriv, config, elapsedTime, arduino_connected_bool
    global Start_PID, min_PWM, max_PWM
    global K, Ti, Td, tf_factor, Tf, setpoint


    while(exiting==False): 
        elapsedTime = time.time() - previous_time 
        if Start_PID == True:
            
            if elapsedTime > deadtime: 
                previous_time = time.time()
                elapsedTime=min(elapsedTime,1)
            
                #PID calcuation
        
                error = setpoint - round(float((PV * openloop_switch)),2)
                lbl_E.config(text=str(round(error,2))) 
       
                if abs(error)<=deadband_error:
                    error=0
                else:
                    pass    
        
                lbl_E2.config(text=str(round(error,2))) 
                lbl_POS.config(text=str(round(float((PV * openloop_switch)),2))) 
          
        
                # computing Proportional controller
                Prop = K * error
            
                # computing Integral controller, variable I_switch determines if the branch is turned on. 
                integ = ((elapsedTime * error) + prev_integ) * I_switch
    
                if D_switch_bool == True or D_kick_bool == True:
                    
                    if D_kick_bool == True and LPF_bool == True:
                        """ Algorithm derivative on PV with low pass filter"""
                        deriv = (((2*K * Td* (-PV+prev_PV)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))#*D_switch
                        # print("D on PV with LPF")
               
                    elif D_switch_bool == True and LPF_bool == True:
                        """ Algorithm derivative on error with LPF"""
                        deriv = (((2*K * Td* (error-previous_error)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))*D_switch
                        # print("D on error with LPF")
                        
                    elif D_kick_bool == True:
                        """ Ideal derivative on PV"""
                        deriv = (((2*K*Td*(-PV+prev_PV)/elapsedTime) - prev_deriv))*D_switch
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
                manipulator = manipulator_list.get()
                if manipulator == "O": 
                    PID = (PID+min_PWM)                    
                if manipulator == "\u03A3":                   
                   PID = (prev_PID + PID)
                if manipulator == "\u0394":                   
                    PID = (PID - prev_PID)
                        
                lbl_MV.config(text=str(round(PID,2)))
    
    
                if PID<=min_PWM:
                    PID=min_PWM
                if PID>=max_PWM:
                     PID=max_PWM   
                PID = round(PID)
                
                lbl_MVL.config(text=str(round(PID)))
                lbl_MVL2.config(text=str(prev_PID))
                          
               
                previous_error = error
                prev_elapsedTime = elapsedTime
                prev_integ = integ
                prev_deriv = deriv
                prev_PID = PID
                prev_PV = PV
    
                try:
                    ser.write(str(chr(round(PID))).encode())
                    ser.flush() # ensure data is transmitted
                    print(f"sending PID pwm value {PID}")
                except:
                    tk.messagebox.showerror("showerror","Error while sending a signal to arduino.")
                    arduino_connected_bool=False
                    Start_PID=False
                    startPID()
                    
        time.sleep(0.005)

                             

def main():
    global previous_time
    previous_time = time.time()
                     

                  
def exit_function():
    global ser, exiting
    
    try:
        exiting=True
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

    
main_Window.protocol('WM_DELETE_WINDOW', exit_function)
systemcalibration_window.protocol("WM_DELETE_WINDOW",showsystemcalibration_window)

fig = Figure(figsize=(DPI_ratio*7.5, DPI_ratio*5.5))          
scope_figure = fig.add_subplot(111)
fig.tight_layout()
scope_figure.grid(True)
line_setpoint,=scope_figure.plot([],[],'b')
line_ballpos,=scope_figure.plot([],[],'r')  
scope_figure.set_xlim(0, scope_mem)
scope_figure.set_ylim(0, tube_length)


class CustomNavigationToolbar(NavigationToolbar2Tk):
    def __init__(self, canvas, parent):
        super().__init__(canvas, parent)

    def save_figure(self, *args):
        global scope_mem_changed
        global runningScope 
        # Override the save_figure method here                
        runningScope=0  
        runningScopeWindow_plot()
        super().save_figure()



scope_figure.legend(["SP",'PV'], loc='upper right',fancybox=True, shadow=False, ncol=1, facecolor='white',frameon=True, framealpha=1,fontsize="16") # ,bbox_to_anchor=(0.38, 0.01)
canvas = FigureCanvasTkAgg(fig, main_Window)
canvas.get_tk_widget().place(x=screenwidthratio*1120, y=screenwidthratio*190)
#toolbar = NavigationToolbar2Tk(canvas, main_Window, pack_toolbar=False)
toolbar = CustomNavigationToolbar(canvas, main_Window)
toolbar.update()
toolbar.place(x=screenwidthratio*1020, y=screenwidthratio*610)
canvas.draw()
canvas.flush_events()    
background=fig.canvas.copy_from_bbox(scope_figure.bbox)


main()

main_Window.lift()


scope_mem_changed=1


thread1=threading.Thread(target=PlotGraph)
thread1.daemon = True
thread1.start()

thread2=threading.Thread(target=plotLines)
thread2.daemon = True
thread2.start()

thread3=threading.Thread(target=read_pos)       
thread3.daemon = True
thread3.start()

thread4=threading.Thread(target=PID_Controller)       
thread4.daemon = True
thread4.start()


tk.mainloop()