# imports
import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import glob
from StringIO import StringIO

################################################################################
# NOTE: TO DO
# - check the grab
# - turn into numpy
# - add a matplotlib output
################################################################################

# parameters -------------------------------------------------------------------
baud_rate = 57600

# helper functions -------------------------------------------------------------
def look_for_available_ports():
    '''
    find available serial ports to Arduino
    '''
    available_ports = glob.glob('/dev/ttyACM*')
    return available_ports

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

def nan_helper(y):
    """Helper to handle indices and logical indices of NaNs.

    Input:
        - y, 1d numpy array with possible NaNs
    Output:
        - nans, logical indices of NaNs
        - index, a function, with signature indices= index(logical_indices),
          to convert logical indices of NaNs to 'equivalent' indices
    Example:
        >>> # linear interpolation of NaNs
        >>> nans, x= nan_helper(y)
        >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
    for more information see:
        http://stackoverflow.com/questions/6518811/interpolate-nan-values-in-a-numpy-array
    """

    return np.isnan(y), lambda z: z.nonzero()[0]

# main class -------------------------------------------------------------------
class grabb_serial_values(object):
    """a class for grabbind data on serial by key
    and turning it into numpy data"""

    def init(self, serial_port_name = 'default'):
        """init the grabber"""

        # set the serial port --------------------------------------------------
        if serial_port_name == 'default':
            # automatically find serial port

            # find available port
            port = look_for_available_ports()

            # if none stop here
            if not port:
                print "No board available"
                return

            # if some, take the first
            print "Using port: "+str(port[0])
            usb_port = serial.Serial(port[0],baudrate=baud_rate,timeout=0.5)
            usb_port.flushInput()

            print "Port imported"

            # set port in library
            self.serial_port = usb_port

        else:
            # use the serial port said by the user

            print "Using port: "+ serial_port_name
            usb_port = serial.Serial(serial_port_name,baudrate=baud_rate,timeout=0.5)
            usb_port.flushInput()

            print "Port imported"

            self.serial_port = usb_port

        # intialize the parsing directories ------------------------------------
        # dictionnary for the data grabbed on the serial
        self.dict_grabbed = {}
        # dictionnary for the data translated to numpy
        self.dict_numpy = {}
        # put the current key at init
        self.current_key = 'init'

        # initialize the key init in the parsing directory ---------------------
        # all data before a char key is received will be logged here
        self.dict_grabbed['init'] = []

        print "Successfull init"

    def parse(self,char_read):
        """parse the current character in the right directory keys"""

        # do nothing if space
        if char_read == ' ':
            pass

        # do nothing if newline
        elif char_read == '\n':
            pass

        # if it is alpha ie letter, should log in the corresponding key list
        elif char_read.isalpha():
            # if already such a key, just use it as the next key and add a
            # comma since starting to log new value
            if char_read in self.dict_grabbed:
                self.current_key = char_read
                self.dict_grabbed[self.current_key].append(',')

            # otherwise, create key and use it as next key
            else:
                # create empty new list for this key
                self.dict_grabbed[char_read] = []
                self.current_key = char_read

        # otherwise number or decimal point: put content in current key
        else:
            self.dict_grabbed[self.current_key].append(char_read)

    def grabb(self,time_grabbing):
        """grab all data for time"""
        time_init = time.time()

        # flush input buffer
        self.serial_port.flushInput()

        print "Start grabbing for set time (seconds): "+str(time_grabbing)

        # do the grabbing during a limited time
        while (time.time() - time_init < time_grabbing):
            if self.serial_port.inWaiting > 0:

                # read one char at a time
                char_read = self.serial_port.read()

                # give it to the parsing routine
                self.parse(char_read)

        # done the parsing for the time asked
        print "Grabbed for the requested time"

    def text_output_dict_grabbed(self):
        """text output of the data grabbed"""

        print self.dict_grabbed
        print "Prnted text output"

    def convert_grabbed_to_numpy(self):
        """convert the grabbed data into numpy and save in the dict_numpy"""

        # if init key is more than 10 chars, text output it

        # for all keys different than init
        for crrt_key in self.dict_grabbed:
            if not crrt_key == 'init':

                # use csv reader to translate string data
                crrt_data = convert_list_feedback(self.dict_grabbed[crrt_key])

                # save in the new dict
                self.dict_numpy[crrt_key] = crrt_data

        print "Converted data to numpy"

    def plot_grabbed_data(self):
        """plot the grabbed data"""

        # parametrize the figure
        plt.figure(figsize=(20,10))

        # for all keys in the numpy directory  plot
        for crrt_key in self.dict_numpy:
            plt.plot(self.dict_numpy[crrt_key],label=crrt_key)

        plt.xlabel('sample')
        plt.ylabel('reading')
        plt.legend()

        plt.show(block=True)

        print "Plotted"

    def clean_numpy_dict(self):
        """cleans the numpy dict for later use and save it:
        - remove the first and last points (may be corrupted)
        - remove the NaN and interpolate
        """

        print "Generate clean data from numpy logged array"

        # create new directory
        self.dict_numpy_cleaned = {}

        #for crrt_key in self.dict_numpy:
        for crrt_key in ['A','B','C','D','E', 'F']:
            # data to clean and save
            crrt_data = self.dict_numpy[crrt_key]
            # remove ends
            crrt_data_no_end = crrt_data[2:-2]
            # find, cound and interpolate NaN
            nans, x= nan_helper(crrt_data_no_end)
            number_of_NaN = np.sum(nans)
            print "Number of NaN found: "+str(number_of_NaN)
            crrt_data_no_end[nans]= np.interp(x(nans), x(~nans), crrt_data_no_end[~nans])

            self.dict_numpy_cleaned[crrt_key] = crrt_data_no_end

    def save_cleaned_dict_numpy(self,path):
        """save the numpy dict for later use"""

        np.save(path,self.dict_numpy_cleaned)

    def save_cleaned_dict_numpy_csv(self,path):
        """save the numpy dict as csv for later use
        first translate into one single matrix for ease of use
        assumes should save the gauges data with keys A, B, C, D, E"""
        
        min_size = min(self.dict_numpy_cleaned['A'].shape[0],self.dict_numpy_cleaned['B'].shape[0],self.dict_numpy_cleaned['C'].shape[0],self.dict_numpy_cleaned['D'].shape[0],self.dict_numpy_cleaned['E'].shape[0],self.dict_numpy_cleaned['F'].shape[0])
        matrix_save = np.zeros((min_size,6))

        matrix_save[:,0] = self.dict_numpy_cleaned['A'][0:min_size]
        matrix_save[:,1] = self.dict_numpy_cleaned['B'][0:min_size]
        matrix_save[:,2] = self.dict_numpy_cleaned['C'][0:min_size]
        matrix_save[:,3] = self.dict_numpy_cleaned['D'][0:min_size]
        matrix_save[:,4] = self.dict_numpy_cleaned['E'][0:min_size]
        matrix_save[:,5] = self.dict_numpy_cleaned['F'][0:min_size]

        # save as csv
        np.savetxt(path+".csv", matrix_save, delimiter=",")

    def return_cleaned_dict_numpy(self):
        """returns the cleaned numpy dict for later use"""

        return self.dict_numpy_cleaned
