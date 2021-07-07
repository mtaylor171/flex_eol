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
PWM_PIN = 19            # GPIO pin 19 for Motor PWM control
MOTOR_EN_PIN = 15       # GPIO pin 15 for Motor enable

FILE_OUTPUT_NAME = str(datetime.datetime.now())
file = open("/home/pi/Documents/" + FILE_OUTPUT_NAME, 'w', newline='')

data = [[],[],[],[],[],[],[],[],[]]
data_single_revolution = [[],[],[],[]]

kDt = 0.5
kAlpha = 0.01
kBeta = 0.0001

def get_us():
    now = datetime.datetime.now()
    return (now.minute*60000000)+(now.second*1000000)+(now.microsecond)

# returns the elapsed time by subtracting the timestamp provided by the current time 
def get_elapsed_us(timestamp):
    temp = get_us()
    return (temp - timestamp)

class MotorController(object):
    SO_FILE = os.path.dirname(os.path.realpath(__file__)) + "/motor_spi_lib.so"
    C_FUNCTIONS = CDLL(SO_FILE)
    INITIAL_US = get_us()
    
    def __init__(self, pwm_pin, motor_pin, motor_duration, pwm_target, mode = GPIO.BOARD, freq = 25000, warnings = False):
        GPIO.setwarnings(warnings)
        GPIO.setmode(mode)
        GPIO.setup(motor_pin, GPIO.OUT)
        self.pwm_pin = pwm_pin
        self.motor_pin = motor_pin
        self.pi = pigpio.pi()
        self.motor_duration = motor_duration
        self.pwm_target = pwm_target
        
        ## Default values
        self.pwm_current = 4
        self.position_hold_time = 0
        self.position_counter = 0
        self.data = []
        self.data_single_revolution = []
        self.last_position = 0
        self.freq_count = [[],[]]
        self.revolution_hold_time = 0

        self.kX1 = 0.0
        self.kV1 = 0.0
        self.x = []
        self.v = []
        self.r = []


    def initialize(self):
    	print("\n\n***********************************\n\n")
        print("Communicating with motor board...\n")
        msg = ""
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 1)
        _self_check = self.C_FUNCTIONS.initialize_motor()

        if not _self_check:
        	print("\nMotor Initialized Successfully\n")

       	else:
            ## TODO Raise exception here
            msg = "ERROR: Could not communicate with motor board. Please disconnect motor."
            return 0, msg
       		
        #if input("Would you like to view the registers? (y/n): ").lower() == 'y':
            #self._read_registers()

            #if(input("\nAre Registers correct? (y/n): ").lower() != 'y'):
                #msg = "Registers Not Selected to be Correct."
                #return 0, msg
        if not self.C_FUNCTIONS.initialize_adc():
            print("\nADC Initialized Successfully\n")
        else:
            msg = "ERROR: ADC Initialize Failed. Please Disconnect motor."
            return 0, msg

        return 1, "Initialization complete!"
    
    def analog_in_initial_send(self):
        self.C_FUNCTIONS.getAnalogInAll_InitialSend()

    # Increases PWM control duty cycle by 1%
    # Gets called by run_main until preferred duty cycle is reached
    def pwm_control(self):
        if(self.pwm_current < self.pwm_target):
            self.pwm_current += 1
            print("PWM: {}".format(self.pwm_current))
        self.pi.hardware_PWM(19, 25000, pwm_current * 10000)

    def bcm2835_init_spi(self):
    	self.C_FUNCTIONS.AD552_Init()

    def bcm2835_motor_ping(self):
    	GPIO.output(self.motor_pin, 1)
    	return self.C_FUNCTIONS.motor_ping()

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
                self.position_counter += 1 
                if(self.position_counter == 6):
                	freq = self._get_rpm(self.revolution_hold_time)
	                self.running_filter(freq)
	                reluctance = self._motor_reluctance(self.x[-1])
                    #rms_val = self._revolution_rms()
                    self.position_counter = 0
                    self.revolution_hold_time = get_us()
                else:
                    #rms_val = 0
                #print("Elapsed: {}, ".format(get_elapsed_us(self.INITIAL_US)) + "Position: {}, ".format(position) + "Frequency: {} ".format(round(freq, 2)) + "Filtered freq: {} ".format(x[-1]) +"PWM: {} ".format(pwm_current) + "Freq/PWM = {} ".format(reluctance) + "RMS Current: {}".format(rms_val))
            else:
            	msg = "INCORRECT POSITION RECORDED"
            	return 0, msg
            self.position_hold_time = get_us()
            self.last_position = position
        else:
            if((get_us() - self.position_hold_time) > 1000000):
                msg = "STALL DETECTED"
                return 0, msg

        return 1, "All Good!"

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
        for duty in range(self.pwm_current, 5, -1):
            self.pi.hardware_PWM(19, 25000, duty * 10000)
            print("PWM: {}".format(duty))
            time.sleep(0.2)
        self.pi.hardware_PWM(19, 0, 0)
        #GPIO.output(self.motor_pin, 0)
        # graph_data()
        #return 0

    def shutdown(self):
    # This occurs when there is a danger event like a stall or overcurrent
    # In this case, we want to shut off everything immediately to prevent further damage
        print("Starting Shutdown")
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 0)
        # graph_data()
        #return 0

	def killall(self):
		self.pi.hardware_PWM(19, 0, 0)
		GPIO.output(MC.motor_pin, 0)

    def motor_results(self, resp, msg):
	    print("\n\n-----------------------------\n")
		print("-----------------------------\n")
		if not resp:
			print("MOTOR FAILED\n")
			print(msg)
		else:
			print("MOTOR PASSED\n")
		print("\n\n-----------------------------\n")
		print("-----------------------------\n")

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
    
    def _get_rpm(self, rev_hold_time):

        freq = 60*( 1000000/(get_us() - rev_hold_time) )
        self.freq_count[0].append(get_elapsed_us(self.INITIAL_US))
        self.freq_count[1].append(freq)
        return freq

    def _motor_reluctance(self, freq):
        return freq/self.pwm_current

    def _revolution_rms(self):
        #TODO: Implement Function here
        return 0

def start_sequence():
	print("*****************************\n\n")
	print(FILE_OUTPUT_NAME)
	print("\n\nNURO MOTOR TESTING\n\n")
	print("*****************************\n\n")

	MC_start = MotorController(PWM_PIN, MOTOR_EN_PIN, 0, 0)

	MC_start.bcm2835_init_spi()

	print("Waiting on motor board to power up...\n")
	print("NOTE: Hold CTRL + 'C' to exit program\n")

	try:
		while(MC_start.bcm2835_motor_ping()):
			pass

		print("\n*****************************\n")
		print("Motor Board Connected!")
		print("\n*****************************\n")

		return 1

	except KeyboardInterrupt:
		end_sequence(MC_start)

		return 0

def end_sequence(MC):
	MC.killall()

def run_motor(MC):
	temp_data = np.uint32([0,0,0,0,0,0,0,0,0])
	adc_reading = 0x0
    index = 0x0
    pwm_counter = 0

	resp, msg = MC.initialize()
	if not resp:
		print(msg + "\n")
		end_sequence(MC)
		return -1, msg

	MC.analog_in_initial_send()

    MC.position_hold_time = MC.revolution_hold_time = get_us()

    while(1):
            if(MC.pwm_current < MC.pwm_target):                              # Ramps up PWM
                if( (pwm_counter == 0) or ((pwm_counter % 1000) == 0) ):
                	MC.pwm_control()
                pwm_counter += 1

            for i in range(0, ACTIVE_CHANNELS):
                data_16bit = MC.get_analog_data() 
                adc_reading, index = data_process(data_16bit)
                temp_data[index+1] = adc_reading
                data[index+1].append(temp_data[index+1])

            temp_data[0] = get_elapsed_us(MC.INITIAL_US)
            data[0].append(temp_data[0])
            writer = csv.writer(file)
            writer.writerow(temp_data)

        try:
            resp, msg = MC.health_check(temp_data)
            if not resp:
            	MC.analog_terminate()
                MC.shutdown()
                return -1, msg
            if(temp_data[0] >= MC.duration * 1000000):
                MC.analog_terminate()
                MC.rampdown()
                return 1, "Motor duration reached"
        except KeyboardInterrupt:

            MC.analog_terminate()
            MC.rampdown()
            msg = "----Keyboard Interrupt by user----"
            return -1, msg

        finally:
            pass



def run_main():
	MOTOR_DURATION_MC1 = 10
	MOTOR_DURATION_MC2 = 10

	MOTOR_PWM_TARGET_MC1 = 5
	MOTOR_PWM_TARGET_MC2 = 85

	MC_1 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC1, MOTOR_PWM_TARGET_MC1)
	MC_2 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC2, MOTOR_PWM_TARGET_MC2)

	print("\n\n\n\n----PLEASE CONNECT MOTOR----\n\n\n")
	
	try:
		while(1):
			if input("Once motor is connected please press 'y' and ENTER: ").lower() == 'y':
				time.sleep(1)
				break
			else:
				print("\n\n*****************************\n")
				print("Incorrect character entered. ")
				print("\n\n*****************************\n\n\n\n")
				pass

		print("\n\n*****************************\n")		
		print("\n----Testing Mode 1----\n\n")

		resp1, msg1 = run_motor(MC_1)
		end_sequence(MC_1)
		if resp1 < 0:
			print(msg1 + "\n")
			print("\n\nRestarting test program...\n\n")
			return -1

		print("\n\n*****************************\n")		
		print("\n----Testing Mode 2----\n\n")

		resp2, msg2 = run_motor(MC_2)
		end_sequence(MC_2)
		if resp2 < 0:
			print(msg2 + "\n")
			print("\n\nRestarting test program...\n\n")
			return -1

		MC_1.motor_results(resp1, msg1)
		MC_2.motor_results(resp2, msg2)

		while(1):
			next_step = input("Press 'c' and ENTER to continue to next motor, or press 'x' and ENTER to exit program: ").lower()
			if next_step == 'y':
				time.sleep(1)
				return 1
			elif next_step == 'x':
				end_sequence(MC_1)
				end_sequence(MC_2)
				return 0
			else:
				print("\n\n*****************************\n")
				print("Incorrect character entered. ")
				print("\n*****************************\n\n")
				pass
	except KeyboardInterrupt:
		end_sequence(MC_1)
		end_sequence(MC_2)
		return 0


def end_sequence():


if __name__ == "__main__":
	while(1):
		if !start_sequence():
			sys.exit()

		while(1):
			state = run_main()

			if state == 0 :
				print("This program will be shutting down in 3 seconds")
				time.sleep(3)
				sys.exit()

			elif state == -1:
				break

			else:
				pass
