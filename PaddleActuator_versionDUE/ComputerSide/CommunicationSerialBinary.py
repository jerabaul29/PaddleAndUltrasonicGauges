"""
********************************************************************************
Protocol:

Implements a simple binary communication on serial port
to be able to transmit more than 8 bits ints as binary
on several bytes. Note, there may be some bits not used in the
bytes finally sent
--------------------------------------------------------------------------------
communication protocol:
[0 or 1]               |  [7 bits]
1: start of new packet |  data
0: continuation        |
--------------------------------------------------------------------------------

Here, this is an even simpler communication protocol: only 2 bytes (first and
last), to transmit only one integer of 14 bits length max

Of course, this technique could be generalize to transmit longer integers (Using
more bytes), of several integers at once, or a data structure, or one of several
data structures (determined by some key following the start bit for example)
********************************************************************************

********************************************************************************
Actuation class:
A class with all methods to easily interact with an Arduino paddle
********************************************************************************

********************************************************************************
Chars used to dialogue with the Arduino board:

##### SENT BY PYTHON TO ARDUINO #####
P: transmit P of PID
I: transmit I of PID
D: transmit D of PID
S: transmit sign of actuation
R: ask if board ready for a new actuation cycle (ie if board at the beginning
   of setup)
X: ask if board ready to receive first buffer, i.e. start execution

##### SENT BY ARDUINO TO PYTHON #####
T: board asking for buffer (ask to transmit)
A: feedcack set point
B: feedback position
C: feedback control
D: feedback milli seconds
Z: actuation is finished
T: total actuation time
U: number of updates set point
V: number of calls of loop
W: number of feedback send
********************************************************************************

********************************************************************************
TO DO:
ADD A SAVER FOR DEFAULT PARAMETERS EXAMPLE PID COEFFICIENTS
WRITE CODE COHERENT PID SET
WRITE CODE COHERENT ACTUATION CYCLE
WRITE SIMPLE COMMAND LINE INTERFACE (MAYBE TERMINAL? OR JUPYTER NOTEBOOK?)
TAKE CARE OF FEEDBACK
OFFER DEFAULT SIGNAL GENERATION PROCEDURES
SEND FREQUENCY CONTROL TO ARDUINO BY SERIAL?
SEND PID LOOP FRQ TO ARDUINO BY SERIAL?
TAKE CARE OF ERROR IF PID TRANSMISSION PROBLEM
********************************************************************************

"""

################################################################################
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import glob
import time
from StringIO import StringIO
################################################################################

################################################################################
# properties of the protocol

# define the number of bits of the int to be sent as two bytes
# note: this should be 14 bits at max (otherwise, need more than
# 2 bytes with this technique)
NUMBER_OF_BITS = 12
# corresponding mean position of the paddle
MEAN_POSITION = 2**(NUMBER_OF_BITS-1)
################################################################################

################################################################################
# properties of the PID values transfer

# defines the number of digits used in the int mantissa (2 to be enough with one
#byte)
NDIGITS_MANTISSA = 2
# define the shifting in the power of 10 to use for transmission of pid
# constants through serial. 128 to be half of one byte.
SHIFT_POWER_OF_10_PID_CSTTS = 128
################################################################################

################################################################################
# properties of the time loops

# frequency of the control signal (scan rate on old paddle)
FREQUENCY_CONTROL = 500

# number of points to send to the board upon buffer request
# NOTE: CAUTION!! the size of the real buffer to send is bigger: if use two bytes, it will
# be twice as many bytes as this number of points!
# the NUMBER_OF_POINTS_PER_BUFFER should be typically half the HALF_INPUT_BUFFER in the Arduino program
# in practise, take a few points less to avoid issues
NUMBER_OF_POINTS_PER_BUFFER = 2038
################################################################################

################################################################################
# put on off additional outputs
DEBUGGING_FLAG = True
################################################################################

################################################################################
# diverse
ONE_MICRO_SECOND = 0.000001
################################################################################

def look_for_available_ports():
    '''
    find available serial ports to Arduino
    '''
    available_ports = glob.glob('/dev/ttyACM*')
    return available_ports

def convert_to_list_bits(value_in,type_out=np.uint16):
    """
    convert value_in into a numpy array of bits
    type_out indicates how many bits are wanted: uint8, uint16
    """
    # convert the input into a numpy array if needed and
    # choose the level of precision
    a = np.array([value_in],dtype=type_out)
    # convert to byte vision
    b = a.view(np.uint8)
    # flip: all less significant bit right
    b = np.flipud(b)
    # unpack into bits
    c = np.unpackbits(b)
    return c

def convert_to_two_bytes(value_bit_format):
    """
    take a value in bit format (16 bits), extract a NUMBER_OF_BITS bit value out
    of it (the less significant bits), and put it in the format corresponding to
    the protocol for transmission of one 10 bits int:
    ----------------------------------------------------------------------------
    16 bits number to be translated in byte1 & byte2
    0 *(16-NUMBER_OF_BITS) | [0 or 1]*NUMBER_OF_BITS

    Max number of bits to be transmitted on two bytes with this method is 14

    byte1:
    1 | 0 * (NUMBER_OF_BITS-14) (empty bits) | [0 or 1] *(NUMBER_OF_BITS - 7) (firts bits of the integer to transmit)

    byte 2:
    0 [0 or 1] * 7        (following bits of the integer to transmit)

    convention: most significant bits first
    """

    # initialize with zeros
    first_byte  = np.unpackbits(np.array(0,dtype=np.uint8))
    second_byte = np.unpackbits(np.array(0,dtype=np.uint8))

    # start of a new packet
    first_byte[0] = 1
    # data
    pivot_1 = 15-NUMBER_OF_BITS
    pivot_2 = 16-NUMBER_OF_BITS
    first_byte[pivot_1:8] = value_bit_format[pivot_2:9]
    second_byte[1:8] = value_bit_format[9:16]

    return np.concatenate((np.packbits(first_byte), np.packbits(second_byte)))

def convert_to_protocol_format(value):
    """
    convert an int into a couple of
    bytes that follow the protocol implementation
    """
    bit_array = convert_to_list_bits(value)
    bytes_array = convert_to_two_bytes(bit_array)

    return bytes_array

"""vectorized version for handling complete buffers directly"""
vectorized_convert_to_protocol_format = np.vectorize(convert_to_protocol_format, otypes=[np.ndarray])

def pid_constant_serial_format(value):
    """
    generate value as a couple mantissa (2 digits int) exponent (base 10):
    value = mantissa * 10 ** exponent
    """

    # power of 10 as an int
    power_of_ten = int(math.floor(math.log10(value*10)))
    # exponent for PID transmission
    # the -NDIGITS_MANTISSA is here because we want a mantissa which is an
    # integer with a given number of digits
    exponent_serial_transmission = power_of_ten-NDIGITS_MANTISSA
    # mantissa
    mantissa = int(math.floor(value*10**(-exponent_serial_transmission)))
    return mantissa, exponent_serial_transmission

def convert_list_feedback(list_feedback):
    """convert a list feedback into a numpy array"""

    # remove the first item: comma
    list_feedback.pop(0)

    # join the list in a string
    list_as_string = ''.join(list_feedback)

    # generate the string
    string_feedback = StringIO(''.join(list_feedback))

    # generate as numpy table
    numpy_feedback = np.genfromtxt(string_feedback, delimiter=",")

    return numpy_feedback

################################################################################

class Paddle_Actuator(object):
    """
    class for interacting with the Arduino controlled paddle actuator
    """

    serial_ready = False

    def set_serial_port(self,serial_port):
        """
        sets the serial port to used
        """
        self.serial_port = serial_port
        self.serial_ready = True

        # wait a bit because the board will reboot when connection is established
        time.sleep(1)

    def connect_to_board(self):
        """
        connect to the board on USB
        """
        # find available port
        port = look_for_available_ports()

        # if none stop here
        if not port:
            print "No board available"
            return

        # if some, take the first
        #print "Using port: "+str(port[0])
        port = '/dev/ttyACM0'
        print "Using port: "+str(port)
        #usb_port = serial.Serial(port,baudrate=57600,timeout=0.5)
        usb_port = serial.Serial(port,baudrate=115200,timeout=0.5)
        #usb_port = serial.Serial(port,baudrate=47600,timeout=0.5)
        usb_port.flushInput()

        print "Port imported"

        # set port in library
        self.set_serial_port(usb_port)

        # return the port for external use if needed
        return(usb_port)

    ############################################################################
    # all methods for transmitting PID parameters

    pid_coefficients_loaded = False
    pid_ready = False

    def set_PID_parameters(self,kp,ki,kd,sign_actuation):
        """
        sets the Kp, Ki, Kd, sign_actuation parameters for the PID controller
        sign_actuation is 0 or 1
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sign_actuation = sign_actuation
        self.pid_coefficients_loaded = True

    def send_one_PID_parameter(self,mantissa,exponent,parameterKey):
        """
        send one PID parameter
        """
        self.serial_port.write(parameterKey)
        char_received = self.serial_port.read()
        if not (char_received == parameterKey):
            print "Received: " + str(char_received)
            print "Problem transmitting PID parameter!"
            #NOTE: do something in that case!!

        self.serial_port.write(chr(mantissa))
        self.serial_port.write(chr(exponent))

        if DEBUGGING_FLAG:
            print "Transmitted: "+str(parameterKey)
            print "Get back:"

        time.sleep(0.1)
        while (self.serial_port.inWaiting() > 0):
            char_feedback = self.serial_port.read()
            if DEBUGGING_FLAG:
                print str(char_feedback)

        if DEBUGGING_FLAG:
            print "Done transmitting"

    def send_sign_actuation(self,sign):
        """
        send actuation sign
        """
        self.serial_port.write('S')
        char_received = self.serial_port.read()
        if not (char_received == 'S'):
            print "Received: " + str(char_received)
            print "Problem transmitting PID parameter!"
            #NOTE: do something in that case!!

        self.serial_port.write(chr(sign))

        if DEBUGGING_FLAG:
            print "Transmitted: S"
            print "Get back:"

        time.sleep(0.1)
        while (self.serial_port.inWaiting() > 0):
            char_feedback = self.serial_port.read()
            if DEBUGGING_FLAG:
                print str(char_feedback)

        if DEBUGGING_FLAG:
            print "Done transmitting"

    def send_PID_parameters(self):
        """
        send all the PID parameters to the Arduino Paddle
        """
        if not self.pid_coefficients_loaded:
            print "PID coefficients were not given to the software!!"
            return False

        # to go through the serial, the constants are expressed as:
        # coefficient (10 to 99) * 10^(exponent+SHIFT_POWER_OF_10_PID_CSTTS)
        # note: transmitt exponent + SHIFT_POWER_OF_10_PID_CSTTS to avoid
        # uint8 sign problems when going through serial!
        mantissa, exponent = pid_constant_serial_format(self.kp)
        self.send_one_PID_parameter(mantissa,exponent+SHIFT_POWER_OF_10_PID_CSTTS,'P')
        #
        mantissa, exponent = pid_constant_serial_format(self.ki)
        self.send_one_PID_parameter(mantissa,exponent+SHIFT_POWER_OF_10_PID_CSTTS,'I')
        #
        mantissa, exponent = pid_constant_serial_format(self.kd)
        self.send_one_PID_parameter(mantissa,exponent+SHIFT_POWER_OF_10_PID_CSTTS,'D')
        #
        # send the sign of actuation
        self.send_sign_actuation(self.sign_actuation)

        # NOTE: add check some feedbach from Actuator here maybe?
        self.pid_ready = True

        return True

    ############################################################################
    # all methods for generation and transmission of buffer
    def set_buffer(self,buffer_values):
        """
        load numpy array values in the buffer
        """
        self.buffer_values = buffer_values

    def generate_buffer_as_bytes(self):
        """
        generate a bytes version of the buffer, ready to transmit to Arduino
        following the communication protocol
        """
        self.buffer_as_bytes = vectorized_convert_to_protocol_format(self.buffer_values)

    def make_next_buffer_ready(self):
        """
        Make nex buffer ready in advance
        """

        # compute end of buffer to generate and check that not out of range
        self.pointer_end_buffer = self.pointer_position_buffer + NUMBER_OF_POINTS_PER_BUFFER
        # -1 to take care of > and 0 starting indexing
        if self.pointer_end_buffer > (self.signal_length-1):
            print 'Hit end of the signal'
            self.end_signal_buffer = True
            self.pointer_end_buffer = self.signal_length-1

        # generate next buffer in advance
        self.set_buffer(self.control_signal[self.pointer_position_buffer:self.pointer_end_buffer])
        self.generate_buffer_as_bytes()

        # update start of next buffer
        self.pointer_position_buffer = self.pointer_position_buffer + NUMBER_OF_POINTS_PER_BUFFER

        # next buffer is ready!
        self.next_buffer_is_ready = True

    def transmit_buffer_bytes_through_serial(self):
        """
        send the buffer bytes (must have been computed before) through the
        serial port
        """

        # for debugging
        #print self.buffer_as_bytes.shape
        #print self.buffer_as_bytes[0]

        for value_to_send in self.buffer_as_bytes:
            # print for debugging
            #print bytes(value_to_send[0])
            #print bytes(value_to_send[1])

            # NOTE: bytes(char(x)) returns x as a byte when 0<=x<256
            self.serial_port.write(bytes(chr(value_to_send[0])))
            self.serial_port.write(bytes(chr(value_to_send[1])))

        # we just transmitted next buffer: next buffer is not ready
        self.next_buffer_is_ready = False

    ############################################################################
    # all methods for loading the signal to be sent to the board
    # the signal to send is stored in an array
    def set_signal(self,signal_array):
        """
        load a numpy array as a control signal
        and add the beginning and ends to have smooth paddle motion
        """
        self.control_signal = signal_array
        self.add_beginning_signal()
        self.add_end_signal()
        self.check_signal()

        self.signal_length = np.size(self.control_signal)
        print 'Length of the signal: '+str(self.signal_length)

        self.signal_ready = True

    # to prevent effect of a distracted user, start smoothly from mean position
    def add_beginning_signal(self):
        """
        add a beginning to the signal so that the paddle starts from the mean
        position smoothly
        """

        #NOTE: to improve if powerful paddle: should then be based on
        # acceleration

        # how much time to go to mean
        # default
        #number_points_go_to_zero = FREQUENCY_CONTROL * 10
        # reduced for faster testing / actuation with good signal
        number_points_go_to_zero = FREQUENCY_CONTROL * 3
        # the offset to correct for
        excess = self.control_signal[0] - MEAN_POSITION
        # how fast go to mean
        decay_base = np.linspace(0,number_points_go_to_zero-1,number_points_go_to_zero)
        time_constant = 5. / number_points_go_to_zero
        decay_array = np.exp(-time_constant*decay_base)
        # decaying return to first set point
        exponention_decay = excess*(1-decay_array) + MEAN_POSITION
        # set first point exact
        exponention_decay[0] = np.array(MEAN_POSITION)

        # concatenate to control signal
        self.control_signal = np.concatenate([exponention_decay,self.control_signal])

    # to prevent effect of a distracted user, end smoothly at mean position
    def add_end_signal(self):
        """
        add an end to the signal so that the paddle comes back to the mean
        position smoothly
        """

        #NOTE: to improve if powerful paddle: should then be based on
        # acceleration

        # how much time to go to mean
        # default
        #number_points_go_to_zero = FREQUENCY_CONTROL * 10
        # reduced for faster testing / actuation with good signal
        number_points_go_to_zero = FREQUENCY_CONTROL * 3
        # the offset to correct for
        excess = self.control_signal[-1]-MEAN_POSITION
        # how fast go to mean
        decay_base = np.linspace(1,number_points_go_to_zero,number_points_go_to_zero)
        time_constant = 5. / number_points_go_to_zero
        decay_array = np.exp(-time_constant*decay_base)
        # decaying return to mean
        exponention_decay = excess*decay_array + MEAN_POSITION
        # set last point exact
        exponention_decay[-1] = np.array(MEAN_POSITION)

        # concatenate to control signal
        self.control_signal = np.concatenate([self.control_signal,exponention_decay])

    def check_signal(self):
        """
        perform basic checks on the signal (range, sign, could add acceleration)
        """

        if np.min(self.control_signal) < 0:
            print "Some negative values in the signal!!"
            return False

        if np.max(self.control_signal) > (2**NUMBER_OF_BITS-1):
            print "Going out of range!!"
            return False

        if not self.control_signal[0] == MEAN_POSITION:
            print "Not starting from mean position!!"
            return False

        if not self.control_signal[-1] == MEAN_POSITION:
            print "Not finishing at mean position!!"
            return False

        # NOTE: could do other checks: acceleration, mean position etc
        # especially if very powerful hydraulic systems

        print "Signal checked: valid"
        return True

    def plot_control_signal(self):
        plt.figure()
        number_points_control = np.size(self.control_signal)
        time = np.linspace(0,number_points_control-1,number_points_control)/FREQUENCY_CONTROL
        #
        plt.plot(time, self.control_signal)
        plt.xlabel('Time (s)')
        plt.ylabel('Signal (int value control)')
        plt.show(block=True)

    ############################################################################
    # all methods for performing one actuation cycle
    def check_ready(self):
        """
        Check that eveything ready for starting one actuation
        Return True if everything is fine
        """

        print "-----------------------------------------------------------------"
        print "PERFORM CHECKS"
        print "-----------------------------------------------------------------"

        if not self.serial_ready:
            print "Serial port not set!!"
            return False

        if not self.pid_coefficients_loaded:
            print "PID coefficients not set in Python code!!"
            return False

        if not self.signal_ready:
            print "Signal not ready!!"
            return False

        #self.serial_port.flushInput()

        print 'Check_ready: everything ready'
        return True

    def perform_setup_and_start(self):
        """
        Perform setup of the Arduino controller for next actuation
        and start by sending the two first buffers
        Return True if everything went fine
        """

        print "-----------------------------------------------------------------"
        print "PERFORM SETUP LOOP"
        print "-----------------------------------------------------------------"

        print 'Starting setup...'

        # first flush buffer
        self.serial_port.flushInput()

        # check that the Arduino board is ready to start a new actuation cycle
        self.serial_port.write('R')
        char_answer = self.serial_port.read()
        if not char_answer == 'R':
            print "Received: "+str(char_answer)
            print 'Arduino is not ready for a new actuation cycle!!'
            return False

        print 'Ready for starting a new actuation cycle'

        # transmitt pid parameters
        print 'Send PID parameters'
        self.send_PID_parameters()

        print 'Check that ready to receive first buffer'
        # check that board waiting for the first buffer
        self.serial_port.write('X')
        char_answer = self.serial_port.read()
        if not char_answer == 'X':
            print 'Arduino is not ready to receive first buffer!!'
            return False

        print "Initialize lists for processing feedback"
        # initialize list for storing feedback
        self.dict_feedback = {}
        self.dict_feedback["feedback_set_point"] = []
        self.dict_feedback["feedback_position"] = []
        self.dict_feedback["feedback_control"] = []
        self.dict_feedback["feedback_time_ms"] = []
        self.dict_feedback["error_msg"] = []
        self.dict_feedback["benign_msg"] = []
        self.dict_feedback["init_trash"] = []
        self.dict_feedback["post_actuation"] = []
        self.dict_feedback["post_actuation_total_actuation_time"] = []
        self.dict_feedback["post_actuation_number_of_updates"] = []
        self.dict_feedback["post_actuation_number_of_loop_calls"] = []
        self.dict_feedback["post_actuation_error_msg"] = []
        self.dict_feedback["number_of_feedback_send"] = []


        print 'Send first double buffer and start actuation'
        # intialize the indice for start of next buffer
        self.pointer_position_buffer = 0
        # NOTE: In arduino, receive the first two buffers in setup to avoid need
        #       for flushing buffer B here
        # make first buffer ready and send it
        self.make_next_buffer_ready()
        self.transmit_buffer_bytes_through_serial()
        # make the second buffer ready and send it
        self.make_next_buffer_ready()
        self.transmit_buffer_bytes_through_serial()

        # reset buffer to avoid trash effect
        #self.serial_port.flushInput()

        # NOW ARDUINO CAN END SETUP AND START LOOP


    def perform_actuation(self):
        """
        core of the actuation, once the actuation has been started through
        perform_setup_and_start
        """

        # NOTE: feedback implemented using ASCII on the way back
        # (Arduino to computer). Not very efficient, but otherwise
        # need protocol

        print "-----------------------------------------------------------------"
        print "PERFORM ACTUATION"
        print "-----------------------------------------------------------------"

        print "Entering actuation core"

        # start actuation
        not_finished = True

        # flag for logging error message
        error_message = False

        # flag for end of the signal buffer
        self.end_signal_buffer = False

        # number of error messages
        self.number_error_messages = 0

        # number of buffers transmitted
        self.number_buffers_transmitted = 2

        # number of feedback received
        self.number_feedback_ms_received = 0

        # read init trash
        current_key = "init_trash"

        while not_finished:

            # check if need to pre generate next buffer and not end of signal yet
            if not self.next_buffer_is_ready:
                if not self.end_signal_buffer:
                    print "A: make next buffer ready"
                    self.make_next_buffer_ready()

            # if a char available on serial, process information from arduino
            if self.serial_port.inWaiting > 0:

                # read one char at a time
                char_read = self.serial_port.read()

                #print "*"+str(char_read)

                # ignore newline
                if char_read == '\n':
                    pass

                # ignore return
                elif char_read == '\r':
                    pass

                # feedback set point follows
                elif char_read == 'A':
                    current_key = "feedback_set_point"
                    self.dict_feedback[current_key].append(',')
                    error_message = False

                # feedback position follows
                elif char_read == 'B':
                    current_key = "feedback_position"
                    self.dict_feedback[current_key].append(',')
                    error_message = False

                # feedback control follows
                elif char_read == 'C':
                    current_key = "feedback_control"
                    self.dict_feedback[current_key].append(',')
                    error_message = False

                # feedback time micro seconds follows
                elif char_read == 'D':
                    current_key = "feedback_time_ms"
                    self.dict_feedback[current_key].append(',')
                    error_message = False
                    self.number_feedback_ms_received = self.number_feedback_ms_received + 1

                # request buffer: serve buffer if not arrived at the end
                elif char_read == 'T' and self.next_buffer_is_ready:

                    print "A: Transmit pre computed buffer"

                    # transmit pre computed buffer
                    self.transmit_buffer_bytes_through_serial()
                    error_message = False
                    self.number_buffers_transmitted = self.number_buffers_transmitted + 1

                # error message (note: use E in the Arduino program only to say error follows)
                elif char_read == 'E':
                    current_key = "error_msg"
                    self.dict_feedback[current_key].append(',')
                    error_message = True
                    self.number_error_messages = self.number_error_messages + 1
                    print "---------------------------- !!RECEIVED ERROR MESSAGE!! ----------------------------"

                # benign message
                elif char_read == 'M':
                    current_key = "benign_msg"
                    self.dict_feedback[current_key].append(',')
                    error_message = False


                # check if the board says actuation is finished
                elif char_read =='Z':
                    not_finished = False
                    error_message = False

                # data about a signal to log in the right list
                else:
                    self.dict_feedback[current_key].append(char_read)
                    if error_message:
                        #print "Error!!"
                        pass

        print "Finished actuation and feedback logging!"
        print "Number of error messages received: "+str(self.number_error_messages)
        print "Number of buffers transmitted: "+str(self.number_buffers_transmitted)

        # log post actuation information ----------------------------------------------------
        # wait a bit to let time to post actuation information to arrive
        time.sleep(0.1)

        while (self.serial_port.inWaiting() > 0):

            ## log
            #self.dict_feedback["post_actuation"].append(self.serial_port.read())

            # read one char at a time
            char_read = self.serial_port.read()

            # ignore newline
            if char_read == '\n':
                pass

            # ignore return
            elif char_read == '\r':
                pass

            # wait for total actuation time
            elif char_read == 'T':
                current_key = "post_actuation_total_actuation_time"
                self.dict_feedback[current_key].append(',')
                error_message = False

            elif char_read == 'U':
                current_key = "post_actuation_number_of_updates"
                self.dict_feedback[current_key].append(',')
                error_message = False

            elif char_read == 'V':
                current_key = "post_actuation_number_of_loop_calls"
                self.dict_feedback[current_key].append(',')
                error_message = False

            elif char_read == 'W':
                current_key = "number_of_feedback_send"
                self.dict_feedback[current_key].append(',')
                error_message = False

            # error message (note: use E in the Arduino program only to say error follows)
            elif char_read == 'E':
                current_key = "post_actuation_error_msg"
                self.dict_feedback[current_key].append(',')
                error_message = True
                self.number_error_messages = self.number_error_messages + 1
                print "---------------------------- !!RECEIVED ERROR MESSAGE!! ----------------------------"

            # data about a signal to log in the right list
            else:
                self.dict_feedback[current_key].append(char_read)
                if error_message:
                    print char_read

        print "Finished post actuation logging!"


    ############################################################################
    # all diagnosis and post processing methods

    def convert_feedback_data(self):
        """
        convert feedback into numpy format
        """

        self.feedback_set_point = convert_list_feedback(self.dict_feedback["feedback_set_point"])
        self.feedback_position = convert_list_feedback(self.dict_feedback["feedback_position"])
        self.feedback_control = convert_list_feedback(self.dict_feedback["feedback_control"])
        self.feedback_time_ms = convert_list_feedback(self.dict_feedback["feedback_time_ms"])
        self.post_actuation_total_actuation_time = convert_list_feedback(self.dict_feedback["post_actuation_total_actuation_time"])
        self.post_actuation_number_of_updates = convert_list_feedback(self.dict_feedback["post_actuation_number_of_updates"])
        self.post_actuation_number_of_loop_calls = convert_list_feedback(self.dict_feedback["post_actuation_number_of_loop_calls"])
        self.number_of_feedback_send = convert_list_feedback(self.dict_feedback["number_of_feedback_send"])


    def analyze_performed_actuation(self):
        """
        analyze of the feedback data and plot it
        """

        print "-----------------------------------------------------------------"
        print "FEEDBACK ANALYSIS"
        print "-----------------------------------------------------------------"

        print "Total actuation time (milli seconds): "+str(self.post_actuation_total_actuation_time)
        print "Total number of set point updates: "+str(self.post_actuation_number_of_updates)
        print "Total number of loop calls: "+str(self.post_actuation_number_of_loop_calls)
        print "Total number of ms feedback send by Arduino: "+str(self.number_of_feedback_send)
        print "Corresponding to a theoretical signal duration (s): "+str(self.number_of_feedback_send/20.)
        print "Total number of ms feedback received: "+str(self.number_feedback_ms_received)
        print "Corresponding to a theoretical signal duration (s): "+str(float(self.number_feedback_ms_received)/20.)

        mean_update_time = self.post_actuation_total_actuation_time / self.post_actuation_number_of_updates
        print "Mean update time (milli seconds): "+str(mean_update_time)
        print "Theory: 2 milli seconds for scan rate 500 Hz"

        mean_loop_time = 1000 * self.post_actuation_total_actuation_time / self.post_actuation_number_of_loop_calls
        print "Mean loop time (micro seconds): "+str(mean_loop_time)

        print "Plot graphical output"

        plt.figure(figsize=(20,10))
        plt.plot(self.feedback_time_ms*ONE_MICRO_SECOND,self.feedback_set_point,label="set point")
        plt.plot(self.feedback_time_ms*ONE_MICRO_SECOND,self.feedback_position,label="position")
        #plt.plot(self.feedback_time_ms*ONE_MICRO_SECOND,self.feedback_control,label="control")

        plt.xlabel('time (s)')
        plt.ylabel('feedback from arduino')

        plt.legend(loc=3)
        plt.show(block=True)
