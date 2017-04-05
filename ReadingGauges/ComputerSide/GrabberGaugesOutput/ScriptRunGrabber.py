import Grabber
import matplotlib.pyplot as plt
%matplotlib inline

gauge_grabber = Grabber.grabb_serial_values()

gauge_grabber.init()
gauge_grabber.grabb(10)
gauge_grabber.convert_grabbed_to_numpy()
gauge_grabber.plot_grabbed_data()
gauge_grabber.clean_numpy_dict()
gauge_grabber.save_cleaned_dict_numpy('test_saving')
