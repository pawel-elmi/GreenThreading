import spidev
from time import sleep
import time
import math
import subprocess
import webapp


import pigpio
# import RPi.GPIO as GPIO
# import threading
from threading import Timer, Thread#, Event
import PID
import numpy as np
#import scipy.interpolate

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# import GUI

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
xs = []
ys = []
zs = []

#fig2 = plt.figure()
#ax2= fig2.add_subplot(1,1,1)

#------------------------ PID TEST ----------------------#
P = 180.0
I = 7
D = 500
PID_SETPOINT = 12.0

pid = PID.PID(P,I,D)
pid.SetPoint = PID_SETPOINT
pid.setSampleTime(0.1)
pid.setWindup(30)
feedback = 0

feedback_list = []
time_list = []
setpoint_list = []

pid_output = 0
#------------------------ END PID TEST ----------------------#


# --------------------- USER SETUP ----------------------- #


_DEVICE_VERSION = {"_0_EXP001_PT100_VERSION__" : [0x00, 0x10+0x01, 0x01, 0x18, 0x10+0x00, 0x81]}
setup_list = _DEVICE_VERSION["_0_EXP001_PT100_VERSION__"]

RetryCounter = 7 #number of seconds to reset if no communication

FILTER_1_CAPAC = 6
FILTER_2_WEIGHT = 16

#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(20, GPIO.OUT)
#GPIO.setup(21, GPIO.OUT)
#GPIO.output(20, GPIO.LOW)
#GPIO.output(21, GPIO.LOW)

_SPI_FREQ = 100000

#_CS_XIN_PWM_PIN = 18
#_CS_XIN_PWM_FREQ = 100000 
#_CS_XIN_PWM_DUTY = 500000

_PWM_PIN = 13
_PWM_FREQ = 5000 
_PWM_DUTY = 5000

_PWM_FANS_PIN = 12
_PWM_FANS_FREQ = 5000
_PWM_FANS_DUTY = 5000

_READER_THREAD_FREQ = 1/10 # how often it reads ADC
_PRINTER_THREAD_FREQ = 1/10 # how often it prints values

Chan_calib_values =  {
    "CH0_unit_1": 10117, #units lower point
    "CH0_res_1": 100,    #ohms lower point
    "CH0_unit_2": 23689, #units upper point
    "CH0_res_2": 150,    #ohms upper point
        "CH1_unit_1": 10132,
        "CH1_res_1": 100,
        "CH1_unit_2": 23810,
        "CH1_res_2": 150,
    "CH2_unit_1": 10190,
    "CH2_res_1": 100,
    "CH2_unit_2": 23850,
    "CH2_res_2": 150,
        "CH3_unit_1": 10129,
        "CH3_res_1": 100,
        "CH3_unit_2": 23780,
        "CH3_res_2": 150,
}
# --------------------- END SETUP ------------------------ #

ChannelsValue = {0: "", 1: "", 2: "", 3: ""} # ADC value dictionary
ChannelFlag = [[0,0],[0,0],[0,0],[0,0]] # _OF, _OD flags
TEMP_RECORDS_LIST = [[],[],[],[]] # list of lists of 6 calculated temperature values to additional filtering
TEMP_RECORDS_LIST_FROZEN = [[],[],[],[]] # list of lists of 6 calculated temperature values to additional filtering
TEMP_CURR = 0
TEMP_DIFF = 3.44
FAKE_TEMP_RECORDS_LIST = [[2,3,4,5,6,7],[2,2,2,2,2,1],[21,21,21],[22,23,22,22,22,22]]
TEMPERATURE = [0,0,0,0] # list of ready, rounded temperatures
FILTER_2_PREV_LIST = [0,0,0,0]

Error_flag = False
IterCounter = 0
#SPI_counter = 0
CheckConnCnt = 0
Frozen_check_value = 30
Frozen_cnt = 0
value = 0
Error_Fatal = False
Error_Connection = False
Error_Connection_Cnt = 0
Error_ADC_Frozen = False
idx = 0

# IterCounter

NR_SETUPS = 4
_READ = 0x08
_WRITE = 0x00
_OF = 0
_OD = 0
_RV = 6
_RS = 0
_LPM = 0
_PS = 0
_PD = 0
_PSS = 0
_DP3_DP0 = NR_SETUPS - 1 
_RC = 1
_LP = 1
_MC = 1
_CFS1_CFS0 = 0
_CONFIG_REG = 0x03
_CHANNEL_SETUP_REG = 0x05

NORMAL_CONVERSION           = 0
SELF_OFFSET_CALIBRATION     = 1
SELF_GAIN_CALIBRATION       = 2
SYSTEM_OFFSET_CALIBRATION   = 5
SYSTEM_GAIN_CALIBRATION     = 6

#SPI Init
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = _SPI_FREQ

#pwm_fans = pigpio.pi()
#pwm_fridge = pigpio.pi()

try: # turns on pigpio deamon to enable pwm for CS5523
#    pwm_cs = pigpio.pi()
#    pwm_cs.hardware_PWM(_CS_XIN_PWM_PIN, _CS_XIN_PWM_FREQ, _CS_XIN_PWM_DUTY) # 100kHz 50% dutycycle
    pwm_fans = pigpio.pi()
    pwm_fans.hardware_PWM(_PWM_FANS_PIN, _PWM_FANS_FREQ, _PWM_FANS_DUTY)
    pwm_fridge = pigpio.pi()
    pwm_fridge.hardware_PWM(_PWM_PIN, _PWM_FREQ, _PWM_DUTY) # 100kHz 50% dutycycle
except:
    while (pwm_fridge.connected == False):
        subprocess.run("sudo pigpiod", check=True, shell=True)
        pwm_fridge = pigpio.pi()
        if(pwm_fridge.connected == True): break
        
def PERFORM_CONVERSION(SETUP): 
    return ((SETUP<<3)| 128)
def PERFORM_CALIBRATION(SETUP,CC):
    return ((SETUP<<3)| 128 | (CC))

def InitInterface():
    for i in range(16):
        spi.writebytes([0xFF])
    spi.writebytes([0xFE])
    
def DecoratorFrame(func):
    def inner1():
        millis = int(round(time.time() * 1000))
        print("=======",millis, "========")
        func()
#        print("==========================")
    return inner1

def WriteBuffer(command, buffer):
    toSend = [command | _WRITE]
    for i in range(len(buffer)):
        toSend.append(buffer[i])    
    spi.writebytes(toSend)

def WriteSetupRegisters():
    WriteBuffer(_CHANNEL_SETUP_REG, setup_list)

def WriteConfiguration(byte1, byte2, byte3):
    buffer = [(_CONFIG_REG | _WRITE)]
    buffer.append(byte1)
    buffer.append(byte2)
    buffer.append(byte3)
    spi.writebytes(buffer)

def ReadRegisters(BitNum, BuffNum):
    to_send = [_CONFIG_REG | _READ]
    spi.xfer(to_send)
    buffer = spi.readbytes(3)
    BitNum = 1 << BitNum
    return (BitNum & buffer[BuffNum])

def SystemReset(): 
    global IterCounter
    global Error_Connection
    global Error_Connection_Cnt
    WriteConfiguration(0,0,0x80) #reset
    if ((ReadRegisters(_RV,2) / int(64)) == 1 and ReadRegisters(1,2) == 0 and ReadRegisters(2,2) == 0) :
        print("Connection Established")
        WriteConfiguration(0,0,0)
        return 0
    else:
        if(IterCounter < RetryCounter):
            Error_Connection_Cnt += 1
            IterCounter += 1
            sleep(1)
            print("N/C cannot reset", IterCounter)
            Init()
        else:
            Error_Connection = True
def ErrGetter():
    global Error_Connection
    global Error_ADC_Frozen
    if Error_Connection == True: return 1
    if Error_ADC_Frozen == True: return 2
    else: return 0
    
def ErrResetter(resetter):
    global Error_Connection
    global Error_ADC_Frozen
    global Frozen_cnt
    if resetter == 1: Error_Connection = 0
    elif resetter == 2: Error_ADC_Frozen = 0
    elif resetter == 3:
        Frozen_cnt += 1
        return Frozen_cnt

def SelfCalibration(setup_num):
    CalibrType = SELF_OFFSET_CALIBRATION
    for CalibrType in range (SELF_GAIN_CALIBRATION):
        to_Send = [PERFORM_CALIBRATION(setup_num,CalibrType)]
        spi.writebytes(to_Send)
        sleep(0.2)
        
def StartConversion():
    to_Send = [PERFORM_CONVERSION(0)]
    spi.xfer(to_Send)
    
def ReadConversion():
    for i in range (4):
        AdcData  = spi.readbytes(3)
        value = AdcData[0]*256 | AdcData[1]
        ChannelNr = int((AdcData[2] & 0xC) / 4)
        ChannelsValue[ChannelNr] = value
        _OF = (AdcData[2] & 0x1)
        _OD = int((AdcData[2] & 0x2) / 2),
        ChannelFlag[ChannelNr][0] = _OF
        ChannelFlag[ChannelNr][1] = _OD 
    return ChannelsValue

def CalcTemp(adc_conversion, chan_num):
    global Error_ADC_Frozen
    global TEMP_DIFF
    try:
        val = int(adc_conversion)# adc_conversion is NaN if CS5523 not powered on
    except ValueError:
        return 0x1233
    
    if(int(adc_conversion) == 65535): return 0x1234 # socket opened / no sensor /
    if(_OF == 1 or _OD == 1): return 0x1235 # overflow or oscillation detected
    
    round_temp = 0.0
    A = 3.9083e-3
    B = -5.775e-7
    
    units_str1  = "CH" + str(chan_num) + "_unit_" + str(1) # string concatenation for query
    resist_str1 = "CH" + str(chan_num) + "_res_"  + str(1)
    
    units_str2  = "CH" + str(chan_num) + "_unit_" + str(2)
    resist_str2 = "CH" + str(chan_num) + "_res_"  + str(2)
    
    units_1 =  Chan_calib_values[units_str1] 
    resist_1 = Chan_calib_values[resist_str1]
    units_2 =  Chan_calib_values[units_str2]
    resist_2 = Chan_calib_values[resist_str2]
    
    a = (units_1 - units_2)/(resist_1 - resist_2)
    b = (units_1 - resist_1*a)*(-1) # linear function coefficients

    R = (b + adc_conversion) / a
    T = 0.0 - A
    T += math.sqrt((A*A) - 4.0*B*(1.0 - R/100))
    T /= 2.0*B
    
    # FILTRAZIONE  
    if len(TEMP_RECORDS_LIST[chan_num]) >= FILTER_1_CAPAC : del TEMP_RECORDS_LIST[chan_num][0] # round temperature value 
    TEMP_RECORDS_LIST[chan_num].append(T)
    
    if len(TEMP_RECORDS_LIST_FROZEN[chan_num]) >= Frozen_check_value: del TEMP_RECORDS_LIST_FROZEN[chan_num][0] #list to check if values are changing
    TEMP_RECORDS_LIST_FROZEN[chan_num].append(T)
    
#    print("Len: ", len(TEMP_RECORDS_LIST[chan_num]))
    
    #FILTER_1
    for i in range(len(TEMP_RECORDS_LIST[chan_num])):
        round_temp += TEMP_RECORDS_LIST[chan_num][i]
        if(TEMP_RECORDS_LIST_FROZEN[chan_num].count(TEMP_RECORDS_LIST_FROZEN[chan_num][0]) == len(TEMP_RECORDS_LIST_FROZEN[chan_num]) and (len(TEMP_RECORDS_LIST_FROZEN[chan_num]) > 5)):
            Error_ADC_Frozen = True
    if len(TEMP_RECORDS_LIST[chan_num]) > 2 : round_temp = (round_temp - min(TEMP_RECORDS_LIST[chan_num]) - max(TEMP_RECORDS_LIST[chan_num])) / (len(TEMP_RECORDS_LIST[chan_num]) - 2)
    #FILTER_2
#    if (len(TEMP_RECORDS_LIST[chan_num]) < 6):
#        round_temp = TEMP_RECORDS_LIST[chan_num]
    if (len(TEMP_RECORDS_LIST[chan_num]) > 6): 
        if len(FILTER_2_PREV_LIST) == 0:
            FILTER_2_PREV_LIST[chan_num].append(round_temp)
        else:
            round_temp = FILTER_2_PREV_LIST[chan_num]*((FILTER_2_WEIGHT - 1) / FILTER_2_WEIGHT) + round_temp*(1 / FILTER_2_WEIGHT)
            FILTER_2_PREV_LIST[chan_num] = round_temp
    TEMP_DIFF = abs(PID_SETPOINT - round_temp)
     
    if (TEMP_DIFF > 1.7):
        pid.ITerm = 0
#        pwm_fans.set_PWM_dutycycle(255)
#    else:
#        pwm_fans.set_PWM_dutycycle(200)
#        I = 1.5
#        if(TEMP_DIFF == 3): pid.ITerm = 50
#    else round_temp = 
    return round_temp
    
def ReadTemperature(): #read conversion, calc to temperature, manage errors
    global CheckConnCnt
    global ERR_flag
    global pid_output
    CheckConnection()
    StartConversion()
    ReadConversion()
    for channel in range(4): 
        buff = (round(CalcTemp(ChannelsValue[channel], channel),2))
        if (buff == 0x1233 or Error_flag == 1 or Error_Connection == True):
            TEMPERATURE[channel] = "ERR - ADC N/C"
            ERR_flag = 1
        elif buff == 0x1234:
            TEMPERATURE[channel] = "ERR - NoSens"
        elif buff == 0x1235:
            TEMPERATURE[channel] = "ERR - OF/OD"
        else:
            TEMPERATURE[channel] = buff
    ERR_flag = 0
    pid.update(TEMPERATURE[0])
    
    pid_output = pid.output
#    print(pid_output)
    if(pid_output < 0):
        if abs(pid_output) > 255: pid_output = 255
        else:
            pid_output = abs(pid_output) 
        pwm_fridge.set_PWM_dutycycle(_PWM_PIN,pid_output)
    else: pwm_fridge.set_PWM_dutycycle(_PWM_PIN,0)
#    feedback_list.append(TEMPERATURE[0])
#    setpoint_list.append(pid.SetPoint)
    
    
@DecoratorFrame
def Printer():
#    f = open("log.txt","a+")
    global TEMP_DIFF
    global TEMPERATURE
    global pid_output
    global idx
    idx += 1
    print("\033c")
    msg = "dupa"
#     webapp.socketio.emit(msg)
    for channel in (0,2,3,1):
        print("CH", channel, ": ", TEMPERATURE[channel],"*C")
    print("Pid output: ", pid_output)
    print("Setpoint: ", PID_SETPOINT)
    print("Idx: ",idx)
    print("P:",round(pid.PTerm,2),"\tI:", round(pid.ITerm,2),"\tD:", round(pid.DTerm,2),"/n")
    print("TEMP_DIFF: ", TEMP_DIFF)

#    pwm_fans.set_PWM_dutycycle(_PWM_FANS_PIN, 255)

#    millis = int(round(time.time() * 1000))
#    string = idx + ". " + millis + pid_output + "\n\r"
#    string = (str(idx) + "\t" + str(round(pid_output, 2)) + "\t" + str(TEMPERATURE[0]) + "\r" ) 
#    f.write(string)
#    f.close()

def CheckConnection():
    InitInterface()
    value = 0
    global IterCounter
    for i in range(8):
        value = ReadRegisters(i,0) + value
    if(value == 255):
        print("System reset")
        IterCounter += 1
        SystemReset()
    else:
        IterCounter = 0
    InitInterface()
    StartConversion()
    return 0

def Init():
    global Error_Connection
    global Error_Fatal
    InitInterface()
    SystemReset()
    if Error_Fatal == True:
        Error_Connection = True
    else:
        WriteConfiguration((_RC | _LP<<1 | _MC<<2 | _CFS1_CFS0<<4),(_LPM | _PS<<1 | _PD<<2 | _PSS<<3 | _DP3_DP0<<4),_RS<<7);
        WriteSetupRegisters()#WriteBuffer(_CHANNEL_SETUP_REG, setup_list)
        InitInterface()
        for i in range (4):
            SelfCalibration(i)
        InitInterface()
        StartConversion()
        
def PlotGraph():
    global idx,xs,ys,zs
#    graph_data = open('log.txt','r').read()
#    lines = graph_data.split('\n')

#    for line in lines:
#        if len(line) > 1:
#            x, y, z = line.split('\t')
#            xs.append(x)
#            ys.append(y)
#            zs.append(z)
    xs.append(idx/10)
    ys.append(TEMPERATURE[0])
    zs.append(round(pid_output, 2))
    
    ax1.clear()
    ax1.plot(xs, ys)
#    print(xs)
#    print(zs)
    ani = animation.FuncAnimation(fig, PlotGraph)
    plt.show()
#    ax2.clear()
#    ax2.plot(xs, ys)

    
    
class ReaderThread(Thread):
    def __init__(self,event):
        Thread.__init__(self)
        self.stopped = event
    def run(self):
        while not self.stopped.wait(_READER_THREAD_FREQ):
            ReadTemperature()
            
class PrinterThread(Thread):
    def __init__(self,event):
        Thread.__init__(self)
        self.stopped = event
    def run(self):
        global Error_Connection_Cnt
        while not self.stopped.wait(_PRINTER_THREAD_FREQ):
            Printer()
            if(Error_Connection_Cnt > 0): print("Connection error detected! CNT:", Error_Connection_Cnt)
            


# Init()
#stopFlag = Event()
#stopFlagPrinter = Event()
#
#ReaderThread(stopFlag).start()
#PrinterThread(stopFlagPrinter).start()

#ReaderThread(stopFlag).setDaemon(True)
#print("ReaderThread.isAlive()", ReaderThread(stopFlag).isDaemon())

#stopFlag.set()
#PrinterThread(stopFlag).setDaemon(True)


#TODO
#=> nie dziala sudo pigpio kiedy tworzy sie 2 obiekty pwm'a

    
#----------------For Calibration ------------------
#try:
#    Init()
#    while True:
#        StartConversion()
#        ReadConversion()
#        channel = 3 # channels 0..3
#        for i in range(4): print("CH: ", channel, ": ", ChannelsValue[channel], "Temp: ", round(CalcTemp(ChannelsValue[channel], channel),2),"*C")
#        sleep(1)
#except KeyboardInterrupt:
#    print("Exit")

#----------------Debug Polling ------------------
#try:
#    Init()
#    while True:
##        StartConversion()
##        ReadConversion()
#        ReadTemperature()
##        for channel in range(4): print(TEMPERATURE[channel])
#        sleep(1)
#        print("\n")
#except KeyboardInterrupt:
#    print("Exit")