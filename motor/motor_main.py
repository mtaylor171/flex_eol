from ctypes import *
import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import csv
import matplotlib.pyplot as plt
import datetime
from datetime import timedelta
import time
import sys
import random
import RPi.GPIO as GPIO
import os
import pigpio


ACTIVE_CHANNELS = 8

x_len = 200
y_range = [0, 3500]

filename = str(datetime.datetime.now())
file = open(filename, 'w', newline='')

FILE_OUTPUT_NAME = str(datetime.datetime.now())

fig, axs = plt.subplots(4)
fig.suptitle('Motor Health')
plt.xlabel('Time (us)')

CHANNELS = 8
pwm_current = 0
position_cntr = 0

code_count = [[],[],[]]
last_position = 0
position_hold_time = 0
freq_count = [[],[]]


data = [[],[],[],[],[],[],[],[],[]]
data_single_revolution = [[],[],[],[]]

x = []
v = []
r = []


kDt = 0.5
kAlpha = 0.01
kBeta = 0.0001
# gets the current time in the form of microseconds - need to revisit this
def get_us():
    now = datetime.datetime.now()
    return (now.minute*60000000)+(now.second*1000000)+(now.microsecond)

# returns the elapsed time by subtracting the timestamp provided by the current time 
def get_elapsed_us(timestamp):
    temp = get_us()
    return (temp - timestamp)

class MotorController(object):
    SO_FILE = os.path.dirname(os.path.realpath(__file__)) + "/ad5592_spi_read.so"
    C_FUNCTIONS = CDLL(SO_FILE)
    INITIAL_US = get_us()
    
    def __init__(self, pwm_pin, motor_pin, mode = GPIO.BOARD, freq = 25000, warnings = False):
        GPIO.setwarnings(warnings)
        GPIO.setmode(mode)
        GPIO.setup(motor_pin, GPIO.OUT)
        self.pwm_pin = pwm_pin
        self.motor_pin = motor_pin
        self.pi = pigpio.pi()
        
        ## Default values
        self.pwm_current = 0
        self.pwm_target = 0
        self.duration = ""
        self.position_hold_time = 0
        self.position_counter = 0
        self.data = []
        self.data_single_revolution = []
        self.last_position = 0
        self.freq_count = [[],[]]

        self.kX1 = 0.0
        self.kV1 = 0.0
        self.x = []
        self.v = []
        self.r = []


    def initialize(self):
    	print("***********************************")
        print("Initializing spi...")
        msg = ""
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 1)
        _self_check = self.C_FUNCTIONS.initialize_motor()
        
        if _self_check:
            ## TODO Raise exception here
            msg = "ERROR: Initialize Failed."
            return 0, msg

        if input("Would you like to view the registers? (y/n): ").lower() == 'y':
            self._read_registers()

            if(input("\nAre Registers correct? (y/n): ").lower() != 'y'):
                msg = "Registers Not Selected to be Correct."
                return 0, msg

        if not self.C_FUNCTIONS.initialize_adc():
            print("ADC Initialized Successfuly")
        else:
            msg = "ERROR: Initialize Failed."
            return 0, msg

        return 1, "Everything Looks Good."


    def pwm_settings(self, duration, pwm_target):
        if duration == "i":
            duration = np.inf
        else:
            try:
                duration = int(duration)
            except ValueError:
                print("invalid duration type")
                return -1

        self.duration = duration
        self.pwm_target = int(pwm_target)

        return 0
    
    def analog_in_initial_send(self):
        self.C_FUNCTIONS.getAnalogInAll_InitialSend()

    # Increases PWM control duty cycle by 1%
    # Gets called by run_main until preferred duty cycle is reached
    def pwm_control(self):
        if(self.pwm_current < self.pwm_target):
            self.pwm_current = self.pwm_current + 1
            print("PWM: {}".format(self.pwm_current))
        self.pi.hardware_PWM(19, 25000, pwm_current * 10000)

    def get_analog_data(self):
        return self.C_FUNCTIONS.getAnalogInAll_Receive()
    
    def analog_terminate(self):
        self.C_FUNCTIONS.getAnalogInAll_Terminate()

    def health_check(self, data):
        code = [0,0,0]
        for i in range(1,4):		# Turning Hall sensor channel data into a 3-digit position code
            if(data[i] > 1650):		# Set a threshold of 1650mV for the hall pulse
                code[i-1] = 1
            else:
                code[i-1] = 0
        position = self._find_positions(code)	# Convert code into a position (1-6)
        if(self.last_position != position):		# Check if position is different from the last recorded position
            if(self.last_position != 0):
                freq = self._get_rpm(self.position_hold_time)
                self.running_filter(freq)
                reluctance = self._motor_reluctance(self.x[-1])
                self.position_counter += 1 
                if(self.position_counter == 6):
                    rms_val = self._revolution_rms()
                    self.position_counter = 0
                else:
                    rms_val = 0
                print("Elapsed: {}, ".format(get_elapsed_us(self.INITIAL_US)) + "Position: {}, ".format(position) + "Frequency: {} ".format(round(freq, 2)) + "Filtered freq: {} ".format(x[-1]) +"PWM: {} ".format(pwm_current) + "Freq/PWM = {} ".format(reluctance) + "RMS Current: {}".format(rms_val))
            else:
            	print("Incorred position recorded")
            	return 0
            self.position_hold_time = get_us()
            self.last_position = position
        else:
            if((get_us() - self.position_hold_time) > 500000):
                print("****WARNING: STALL DETECTED****") 
                return 0

        return 1

    def running_filter(self, data):
        x_k = self.kX1 + kDt * self.kV1
        r_k = data - x_k
        x_k = x_k + kAlpha * r_k
        v_k = self.kV1 + (kBeta/kDt) * r_k

        self.kX1 = x_k
        self.kV1 = v_k

        self.x.append(x_k)
        self.v.append(v_k)
        self.r.append(r_k)        

    def rampdown(self):
        print("Starting rampdown...")
        for duty in range(self.pwm_current, -1, -1):
            self.pi_pwm.ChangeDutyCycle(duty)
            print("PWM: {}".format(duty))
            time.sleep(0.5)
        GPIO.output(self.motor_pin, 0)
        # graph_data()
        return 0

    def shutdown(self):
    # This occurs when there is a danger event like a stall or overcurrent
    # In this case, we want to shut off everything immediately to prevent further damage
        print("Starting Shutdown")
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 0)
        # graph_data()
        return 0


    def _read_registers(self):
    # Reads all registers on DRV8343 and prints them
        for i in range(19):
            reg_data = self.C_FUNCTIONS.motor_register_read(i)
            print('Register {}:'.format(i) + ' {}'.format(hex(reg_data)));
            print('\n')

    def _find_positions(self, code):
    # Converts the hall sensor pulse data into a position (1-6)
    # If the hall sensor pulses do not align with one of these positions, a zero is returned at which there will a flag raised
        if code == [1, 0, 1]:
            return 1
        elif code == [0, 0, 1]:
            return 2
        elif code == [0, 1, 1]:
            return 3
        elif code == [0, 1, 0]:
            return 4
        elif code == [1, 1, 0]:
            return 5
        elif code == [1, 0, 0]:
            return 6
        else:
            return 0
    
    def _get_rpm(self, pos_hold_time):

        freq = (1000000/((get_us() - pos_hold_time)*6))
        self.freq_count[0].append(get_elapsed_us(self.INITIAL_US))
        self.freq_count[1].append(freq)
        return freq

    def _motor_reluctance(self, freq):
        return freq/self.pwm_current

    def _revolution_rms(self):
        #TODO: Implement Function here
        return 0


# Processes the raw ADC data by multiplying by factor of 4096/5000 = 0.819
# Also extracts the index from data frame
# Returns both of these
def data_process(data):
    adc_reading = int((data & 0x0FFF) / 0.819)
    index = ((data >> 12) & 0x7)
    return adc_reading, index

# Graphs the current data. This is currently screwed up
def graph_data():
    #filter_data(freq_count[1])
    axs[0].plot(freq_count[0], x)
    axs[1].plot(data[0], data[4])
    axs[2].plot(data[0], data[5])
    axs[3].plot(data[0], data[6])
    plt.show()

# This is the main script which commands the pwm, adc data, and 
# The while loop will keep running until time elapses, a keyboard interrupt, or the motor stalls

def run_main():
    PWM_PIN = 19
    MOTOR_EN_PIN = 15
    
    MC = MotorController(PWM_PIN, MOTOR_EN_PIN)
    resp, msg = MC.initialize()
    print("***********************************")

    if not resp:
        print(msg)
        return -1
    
    resp = MC.pwm_settings(
        input("Enter sample duration (type 'i' for inifinite): "),
        input("Enter target duty cycle % (0-100):"))

    if not resp:
        print("PWM Settings incorrect. Exiting")
        return -1

    print("***********************************")
    temp_data = np.uint32([0,0,0,0,0,0,0,0,0])
    adc_reading = 0x0
    index = 0x0
    pwm_counter = 0
    MC.analog_in_initial_send()                                         # Sends initial command to ADC to start recording all channels repeatedly
    position_hold_time = get_us()										# Gets initial timestamp for position time tracking

    while(1):
            if((pwm_counter % 1000) == 0):
                MC.pwm_control()											# Adjusts PWM for ramp-up
            pwm_counter = pwm_counter + 1								# Counter allows for a gradual ramp-up
            for i in range(0, ACTIVE_CHANNELS):
                data_16bit = MC.get_analog_data() 
                adc_reading, index = data_process(data_16bit)
                temp_data[index+1] = adc_reading
                data[index+1].append(temp_data[index+1])
            temp_data[0] = get_elapsed_us(MC.INITIAL_US)
            data[0].append(temp_data[0])
            #print('Time Elapsed: {}'.format(temp_data[0]))
            writer = csv.writer(file)
            writer.writerow(temp_data)
        try:
            resp = MC.health_check(temp_data)
            if not resp:
                MC.shutdown()
                raise Exception("Health Check Failed")
            if(MC.duration != np.inf):
                if(temp_data[0] >= MC.duration * 1000000):
                    MC.analog_terminate()
                    MC.rampdown()
        except KeyboardInterrupt:

            MC.analog_terminate()
            MC.rampdown()

        finally:
            pass





if __name__ == "__main__":
    run_main()