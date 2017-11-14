# BBIE stands for Behavior Based Interval Estimation

import intervalestimation
import copy
import os
import logging
import util.nullhandler
import pickle
from random import shuffle
import csv
import time
import subprocess
import body.bodycontroller as body

logging.getLogger('Borg.Brain.BBIE').addHandler(util.nullhandler.NullHandler())
'''
class BBIE(object):
    def __init__(self, file_name = "bbie.dat"):
        self.logger = logging.getLogger('Borg.Brain.BBIE')
        self.file_name = file_name
        self.ie = intervalestimation.IntervalEstimation()
        self.dictionary = self.read_data( )

    def read_data(self):
        """read the data from a file
        the format in the file is the following:
        behavior_item_item 12,543,1234,6534,654,87,9,0,6
        """
        try:
            all_data = open ( self.file_name, 'r' ).readlines()
            data_dictionary = {}
            for dat in all_data:
                d = dat.split(":")
                data_dictionary[ eval(str(d[0])) ] = eval(str(d[1:][0]))
            return data_dictionary
        except:
            no_data = open( self.file_name, 'w' )
            no_data.close()
            return {}

    def update_dictionary(self, key, value):
        try:
            vals = self.dictionary[key]
            vals+=(value)
            self.dictionary[key] = vals
        except:
            self.dictionary[key] = [value]
        return self.dictionary


    def write_data(self):
        try:
            f_out = open(self.file_name, 'w')
            for key in self.dictionary.keys():
                output = "'" + key + "'" + ':' + str(self.dictionary[key]) + '\n'
                f_out.write( output )
            f_out.close()
        except:
            print '\n\nError 42: writing to write ' + str(self.file_name) + 'failed'
            print 'check permissions of user'
            print 'Error 42 is generated in bbie.py\n\n'

    def clear_file(self):
        os.system('rm ' + str(self.file_name)) '''

    ##############################################################
    ### Methods for multiple-version behaviour learning.
    ###

def get_best_implementations(file_loc):
    '''
    Method which tries to recover learned multiple-version behaviours and return an ordered list of (execution time, id tuples).
    If no multiple-version learning file is detected for the behavior, will respond with (ridiculously long, 1).

    params contains the relevant info for finding the file etc:
    TODO
    '''
    try:
        # Try finding the file where the learning data is stored.
        input_file = open(file_loc)
        learning_data = pickle.load(input_file)
        input_file.close()
        behavset = learning_data['behaviors']
        print ("BBIE file read; location: "+file_loc)
        if (not learning_data['training_completed']):
            # The behaviors have not finished basic training; some versions have not been executed the minimum number of times.
            opts = []
            for el in behavset:
                if learning_data['use_lower']:
                    if (el['times_run']<learning_data['min_to_train']):
                        opts+=((learning_data['max_duration'], el['id']))
                else:
                    if (el['times_run']<learning_data['min_to_train']):
                        opts+=((learning_data['max_duration'], el['id']))
            if (opts == []):
                # Training was completed, start using the normal procedure.
                learning_data['training_completed'] = True
                # And store the fact that training was completed.
                output=open(file_loc, 'wb')
                pickle.dump(learning_data, output)
                output.close()

            else:
                # Shuffle these and return.
                shuffle(opts)
                print "Using BBIE for behavior " + learning_data['behavior_name'] + ", versions provided:"
                print opts
                return opts

        # The behaviors have completed their basic training, follow default procedure.
        # Get the relevant bounds with their respective ID's.
        print "Trying to use learned data."
        opts = []

        for el in behavset:
            if learning_data['use_lower']:
                opts+=((el['lower'], el['id']))
            else:
                opts+=((el['upper'], el['id']))
        # Sort these appropriately using the boundaries.

        if learning_data['use_lower']:
            opts = sorted(opts)
        else:
            opts = sorted(opts, reverse=True)
        # Replace the bounds with the max duration.

        time_replaced = []
        for el in opts:
            bound, behav_version = el
            time_replaced+=((learning_data['max_duration'], behav_version))

        relevant = time_replaced[0:learning_data['top_nr']]

        print "Using BBIE for behavior " + learning_data['behavior_name'] + ", versions provided:"
        print relevant

        return relevant

    except:
        # Just return (a day, 1).
        #print "Something went wrong reading the BBIE file!"
        return [(86400, 1)]

def save_results(version_id, file_loc, duration, finished, failed):
    '''
    Method which stores info when a file is timed out or completed.
    This info is immediately used to update the interval estimation for the version just used.

    id is the number of the version used.
    params is a dict with relevant parameters.
    '''

    # Store success/failure in CSV file
    store_in_csv(version_id, file_loc, duration, finished, failed, 0)

    try:
        # Try to find the file where data should be stored.
        input_file = open(file_loc)
        learning_data = pickle.load(input_file)
        input_file.close()
        behavset = learning_data['behaviors']
    except:
        # No file found, register nothing.
        #print "Failed to save to: " + file_loc
        return

    # Get correct element from behavset.
    goal = None

    for el in behavset:
        # Check if this is the used version.
        if (el['id']==version_id):
            goal = el
            break

    if (goal==None):
        raise Error("Target element was not found while trying to save.")

    # Goal is the element we want to use.

    # Apply Interval Estimation to new data.
    ie = intervalestimation.IntervalEstimation()

    if (not failed):
        goal['completion_times']+=(duration)
        goal['times_run'] =  goal['times_run']+1
        if (finished):
            goal['non_cutoff_times']+=(duration)
            goal['times_completed'] =  goal['times_completed']+1

    if (failed):
        goal['failed_times']+=(duration)
        goal['times_failed'] =  goal['times_completed']+1
        if(learning_data['use_lower']):
            goal['completion_times']+=(learning_data['max_duration'])
        else:
            goal['completion_times']+=(0)
        goal['times_run'] =  goal['times_run']+1

    if (goal['times_run']>1):
        goal['upper'] = ie.get_upperbound(learning_data['confidence_level'], goal['completion_times'])
        goal['lower'] = ie.get_lowerbound(learning_data['confidence_level'], goal['completion_times'])

    # Store modified data.
    output=open(file_loc, 'wb')
    pickle.dump(learning_data, output)
    output.close()
    print "Number of times run: "+str(goal['times_run'])
    print "Data stored successfully!"

def check_tries(version_id, file_loc, num_tries):
    '''
    Method to check if a behavior has had too may tries already.
    If no BBIE is used, there is a limit of 1 try.
    If BBIE is used, there is a limit set in the .pkl file.
    '''
    try:
        # Try to find the file where data should be stored.
        input_file = open(file_loc)
        learning_data = pickle.load(input_file)
        input_file.close()
    except:
        # No file found, register nothing.
        return True

    max_tries = learning_data['max_tries']
    if(num_tries>=max_tries):
        return True
    else:
        return False

def store_in_csv(version_id, file_loc, duration, finished, failed, script, failure_reason="none"):
    # Strip behavior name from file_loc
    behav_name = file_loc
    # Remove anything in fromt of a '/'
    while '/' in behav_name:
        _, behav_name = behav_name.split('/',1)
    # Remove everything after '.'
    try:
        behav_name, _ = behav_name.split('.',1)
    except:
        pass
    # Add the line to behavior_completions.csv.
    write_target = open(os.environ['BORG'] + '/brain/data/behavior_completions.csv', 'ab')
    
    writer = csv.writer(write_target, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    writer.writerow([behav_name, version_id, duration, finished, failed, script, time.localtime(time.time())])
        
    write_target.close()
    
    # State the behavior finished.
    #completion_message = ""
    #if script>0:
    #    completion_message+=("Plan")
    #else:
    #    completion_message+=("Action")
    #completion_message+=(" ")
    #completion_message+=(behav_name)
    #
    #if finished>failed:
    #    completion_message+=(" finished successfully.")
    #else:
    #    completion_message+=(" failed. The reason given was: ")
    #    completion_message+=(failure_reason)
    
    #try:
    #    speech_body
    #except:
    #    speech_body = body.BodyController()
    #speech_body.say(completion_message)
    
    # Publish to ROS tracking
    #try:
    #    if (script>0):
    #        is_script = True
    #    else:
    #        is_script = False
    #    test_log = "rostopic pub -1 /borg_state borg_interface/Borg_behav_state  -- 2 " + str(behav_name) + " " + str(version_id) + " " + "0" + " " + str(is_script)
    #    subprocess.Popen(test_log, shell=True)
    #except:
    #    print "Something went wrong logging to ROS!"
    
def store_start_in_csv(version_id, file_loc, depth, script):
    # Strip behavior name from file_loc
    behav_name = file_loc
    # Remove anything in fromt of a '/'
    while '/' in behav_name:
        _, behav_name = behav_name.split('/',1)
    # Remove everything after '.'
    try:
        behav_name, _ = behav_name.split('.',1)
    except:
        pass
        
    #DEBUG
    #print "Logging init for: " + str(behav_name)
    
    # Add the line to behavior_completions.csv.
    write_target = open(os.environ['BORG'] + '/brain/data/behavior_starts.csv', 'ab')
    
    writer = csv.writer(write_target, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    writer.writerow([behav_name, version_id, script, time.localtime(time.time())])
        
    write_target.close()
    
    # State the behavior started.
    #completion_message = ""
    #if script>0:
    #    completion_message+=("Plan")
    #else:
    #    completion_message+=("Action")
    #completion_message+=(" ")
    #completion_message+=(behav_name)
    #completion_message+=(" was set up.")
        
    #try:
    #    speech_body
    #except:
    #    speech_body = body.BodyController()
    #speech_body.say(completion_message)
    
    # Publish to ROS tracking
    #try:
    #    if (script>0):
    #        is_script = True
    #    else:
    #        is_script = False
    #    test_log = "rostopic pub -1 /borg_state borg_interface/Borg_behav_state  -- 0 " + str(behav_name) + " " + str(version_id) + " " + "0" + " " + str(is_script)
    #    subprocess.Popen(test_log, shell=True)
    #except:
    #    print "Something went wrong logging to ROS!"
    
def store_precon_in_csv(version_id, file_loc, depth, script):
    # Strip behavior name from file_loc
    behav_name = file_loc
    # Remove anything in fromt of a '/'
    while '/' in behav_name:
        _, behav_name = behav_name.split('/',1)
    # Remove everything after '.'
    try:
        behav_name, _ = behav_name.split('.',1)
    except:
        pass
    
    #DEBUG
    print "Logging precon for: " + str(behav_name)
    
    # Add the line to behavior_completions.csv.
    write_target = open(os.environ['BORG'] + '/brain/data/behavior_precon.csv', 'ab')
    
    writer = csv.writer(write_target, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    writer.writerow([behav_name, version_id, script, time.localtime(time.time())])
        
    write_target.close()
    
    # State the behavior started running.
    #completion_message = ""
    #if script>0:
    #    completion_message+=("Plan")
    #else:
    #    completion_message+=("Action")
    #completion_message+=(" ")
    #completion_message+=(behav_name)
    #completion_message+=(" has started running.")
        
    #try:
    #    speech_body
    #except:
    #    speech_body = body.BodyController()
    #speech_body.say(completion_message)
    
    # Publish to ROS tracking
    #try:
    #    if (script>0):
    #        is_script = True
    #    else:
    #        is_script = False
    #    test_log = "rostopic pub -1 /borg_state borg_interface/Borg_behav_state  -- 1 " + str(behav_name) + " " + str(version_id) + " " + "0" + " " + str(is_script)
    #    subprocess.Popen(test_log, shell=True)
    #except:
    #    print "Something went wrong logging to ROS!"
