# Motor Bringup 

## Description
###### This folder contains the code which will be used for testing and bringup of the motor

## Compiling C library
The C code must be compiled as an .so in order for the python ctypes library to access it.

**To compile:**

*gcc -o -shared motor_spi_lib.so motor_spi_lib.c -l bcm2835*

***NOTE:*** In order to compile and run this code, the RPi must have the bcm2835 library installed on it

## Running Python Script
**To run script:**

*sudo python3 motor_main.py*

Alternatively, the Raspberry Pi that the test team will get has a desktop icon that they can simply double click on and then click "Execute"

The script has some user-defined fields that the testers will need to fill in:

*'Enter sample duration:'* - for this please enter how many seconds you would like to run the motor. If you would like to run it infinitely, type 'i'
*'Enter target duty cycle:'* - this is the duty cycle percentage that the PWM will be controlling the motor. This value can range from 0-100. We are hoping to add functionality to change the PWM mid-run in the future
