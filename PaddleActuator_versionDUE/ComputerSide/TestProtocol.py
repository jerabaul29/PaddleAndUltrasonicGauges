#%reset -f
#%matplotlib inline

import serial
import CommunicationSerialBinary
import numpy as np
import tictoc
import glob
import time
import SignalGeneration

# ------------------------------------------------------------------------------
# NOTE:
# to do:
# ------------------------------------------------------------------------------

################################################################################
# example of saving signal to a file
# not: does not work in an Hydrogen notebook because of raw_imput not available in this backend
signal_to_save = np.zeros((1000,))
np.savez('Example_np_signal_saved',signal=signal_to_save)
# example of loading a signal from a file
signal_class = SignalGeneration.signal_generation()
signal_class.import_signal()

################################################################################
# wait a bit
time.sleep(5)

################################################################################
print "########################## STARTING ##########################"
communication_serial = CommunicationSerialBinary.Paddle_Actuator()

################################################################################
# find available port and connect to it
usb_port = communication_serial.connect_to_board()

################################################################################
# set the actuation signal
# scan rate in Hz
scan_rate = 500.
frequency = 2.
amplitude = 950.
mean_position = 2048.
time_seconds = 60.

signal_class = SignalGeneration.signal_generation()
signal_class.generate_time_base(time_seconds, scan_rate)

signal_class.generate_sinusoidal_signal(amplitude,frequency,mean_position)

signal = signal_class.return_signal()

################################################################################
# using the library to set a signal
small_value = 0.0000001
direction_actuation = 0
# SET THE PID PARAMETERS: P  |  I  |  D  |  S
# NOTE: never put zero, otherwise transfer protocol fails (use small_value instead)
# old values from Paddle UNO
#communication_serial.set_PID_parameters(8,2,0.07,1)
communication_serial.set_PID_parameters(2,2,0.04,direction_actuation)

# SET THE SIGNAL AND PLOT ------------------------------------------------------
communication_serial.set_signal(signal)
communication_serial.plot_control_signal()
time.sleep(0.5)

# CHECK THAT EVERYTHING IS READY -----------------------------------------------
communication_serial.check_ready()

# PERFORM SETUP AND START ACTUATION --------------------------------------------
communication_serial.perform_setup_and_start()

# perform actuation
communication_serial.perform_actuation()

# post actuation checks and analysis -------------------------------------------

communication_serial.dict_feedback["init_trash"]
communication_serial.dict_feedback["error_msg"]
communication_serial.dict_feedback["feedback_set_point"]

communication_serial.convert_feedback_data()
communication_serial.analyze_performed_actuation()

################################################################################
print "######################### FINISHING ##########################"




# end
