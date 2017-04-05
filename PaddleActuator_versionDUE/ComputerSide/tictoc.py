#Homemade version of matlab tic and toc functions

def tic(print_message=False):
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()
    if print_message:
        print "Reset tictoc"

def toc(print_message=False):
    import time
    if 'startTime_for_tictoc' in globals():
        tictoc_delay = time.time() - startTime_for_tictoc
        if print_message:
            print "Elapsed time is " + str(tictoc_delay) + " seconds."
        return (tictoc_delay)
    else:
        if print_message:
            print "Toc: start time not set"
