# script to call for easy end-use actuation

import serial
import CommunicationSerialBinary
import numpy as np
import tictoc
import glob
import time
import SignalGeneration

# which actuator to use
#fast_actuator = True # to use with the long and thin actuator
fast_actuator = True # to use with the shorter and thicker actuator

# global parameters ------------------------------------------------------------
# scan rate in Hz
scan_rate = 500.
# mean position, ie 2**(12-1)
mean_position = 2048.

# header -----------------------------------------------------------------------
print "************************************************************************"
print "*                   PADDLE ACTUATOR v2.0                               *"
print "Written by Jean Rabault, PhD student, University of Oslo"
print "Designed for with Arduino Due and Moto control shield"
print "More details at: https://github.com/jerabaul29"
print "Please contact me before sharing or using the code"
print "************************************************************************"
print " "

# initialize the library -------------------------------------------------------
print "Initialize library -----------------------------------------------------"
communication_serial = CommunicationSerialBinary.Paddle_Actuator()
usb_port = communication_serial.connect_to_board()

# set the PID parameters -------------------------------------------------------
print "Set PID parameters -----------------------------------------------------"
if fast_actuator:
    print "Default: P: 0.7 | I: 0.7 | D: 0.005 | S: 0"
else:
    print "Default: P: 1.8 | I: 0.7 | D: 0.005 | S: 0"

use_default_params = raw_input("Use Default parameters (Yes [y] / No [n])")

if use_default_params == 'y':
    if fast_actuator:
        communication_serial.set_PID_parameters(0.7,0.7,0.005,0)
    else:
        communication_serial.set_PID_parameters(1.8,0.7,0.005,0)
elif use_default_params == 'n':
    # ask user for parameters
    print "Enter parameters; P, I and D must be strictly positive"
    P = float(raw_input("P: "))
    I = float(raw_input("I: "))
    D = float(raw_input("D: "))
    S = int(raw_input("S (0 or 1): "))
    communication_serial.set_PID_parameters(P,I,D,S)
else:
    print "invalid input, abort"
    quit()

# perform as many actuations as the user wants ---------------------------------
perform_one_actuation = True

while perform_one_actuation:

    print "************************************************************************"

    # SET THE SIGNAL
    print "Set the signal to send to paddle"
    choice_signal = raw_input("1: sinusoidal signal; 2: modulated signal; 3: read from file | ")

    if choice_signal == '1' or choice_signal == '2':
        print "Sinusoidal signal must have amplitude no more than 2048"
        frequency = float(raw_input("frequency (Hz): "))
        amplitude = float(raw_input("amplitude     : "))
        duration  = float(raw_input("duration (s)  : "))

        if choice_signal == '1':
            signal_class = SignalGeneration.signal_generation()
            signal_class.generate_time_base(duration, scan_rate)
            signal_class.generate_sinusoidal_signal(amplitude,frequency,mean_position)
            signal = signal_class.return_signal()

        if choice_signal == '2':
            modulation = float(raw_input("Modulation    : "))

            signal_class = SignalGeneration.signal_generation()
            signal_class.generate_time_base(duration, scan_rate)
            signal_class.generate_modulated_signal(amplitude,frequency,mean_position,modulation)
            signal = signal_class.return_signal()

    elif choice_signal == '3':
        signal_class = SignalGeneration.signal_generation()
        signal_class.import_signal()
        signal = signal_class.return_signal()

    communication_serial.set_signal(signal)

    # CHECK THAT EVERYTHING IS READY -------------------------------------------
    communication_serial.check_ready()

    # PERFORM SETUP AND START ACTUATION ----------------------------------------
    communication_serial.perform_setup_and_start()

    # PERFORM ACTUATION --------------------------------------------------------
    communication_serial.perform_actuation()

    # POST ACTUATION CHECKS AND ANALYSIS ---------------------------------------

    communication_serial.convert_feedback_data()
    communication_serial.analyze_performed_actuation()

    # ASK FOR WHAT TO DO NEXT --------------------------------------------------
    print "-----------------------------------------------------------------"

    continue_answer = raw_input("Run one more actuation? (Yes [y] / No [n])")

    if continue_answer == 'y':
        pass

    elif continue_answer == 'n':
        perform_one_actuation = False

    else:
        print "Wrong input; stop here"
        quit()
