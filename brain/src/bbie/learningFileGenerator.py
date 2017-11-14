from copy import deepcopy
import pickle
'''
This is the storage file generator ful multiple-version behaviour learning.

To generate a storage file, import this file and run create_ML_file(args)

!Remember to re-import if you ran this with non-default values for some of the arguments!
!Otherwise, things may not work as you intend!
'''

def create_ML_file(goal, behavior_name, id_list, max_duration, top_nr, use_lower=True, confidence_level=95, min_to_train=5, max_tries = 10):
    '''
    goal is the location of the file to be written to.
    behavior_name is the name fo the behavior the file is for, without the underscore of the number.
    id_list is a list of behavior id's to use/learn for.
    max_duration is a maximum cutoff time for the behavior
    top_nr is the number of results which should be returned when selecting behaviors.
    use_lower determines if you use the lower bound of the confidence interval (faster = better) or the upper one (longer = better)
    confidence_level is the confidence level for the interval in percents (95 for 95%).
    min_to_train is the minimum number of times each behavior has to be executed before training is considered complete.
    '''
    # Check the inputs.
    if not isinstance(goal, str):
        raise SyntaxError("goal should be a string!")
    if not isinstance(behavior_name, str):
        raise SyntaxError("behavior_name should be a string!")
    if not isinstance(id_list, list):
        raise SyntaxError("idlist should be a list!")
    if not isinstance(use_lower, bool):
        raise SyntaxError("use_lower should be a boolean!")
    if not (isinstance(confidence_level, int) or isinstance(confidence_level, float)):
        raise SyntaxError("confidence_level should be an int or float!")
    if not isinstance(top_nr, int):
        raise SyntaxError("top_nr should be an int!")
    if not isinstance(max_duration, int):
        raise SyntaxError("max_duration should be an int!")
    if not isinstance(min_to_train, int):
        raise SyntaxError("min_to_train should be an int!")

    # Generate the list of dicts for the behavior versions.
    behaviors = range(len(id_list))
    for i in behaviors:
        behaviors[i] = {'id': id_list[i], 'completion_times': [], 'non_cutoff_times': [], 'failed_times': [], 'upper': deepcopy(max_duration), 'lower': 0, 'times_completed': 0, 'times_run': 0, 'times_failed': 0}

    # Add this list and the other values to a dict.
    usedDict = {'behavior_name': behavior_name, 'behaviors': behaviors, 'use_lower': use_lower, 'confidence_level': confidence_level, 'top_nr': top_nr, 'max_duration': max_duration, 'training_completed': False, 'min_to_train': min_to_train, 'max_tries': max_tries}

    # And create the file.
    output=open(goal, 'wb')
    pickle.dump(usedDict, output)
    output.close()

def get_behavior_numbers():
    """Used to get the list of versions to use."""
    nr_of_versions = eval(raw_input("How many versions should be used? "))

    versions_list = []
    for el in range(nr_of_versions):
        el = raw_input("Version " + str(el) + ": ")
        versions_list.append(eval(el))

    # And return the generated list.
    return versions_list

def main():
    """Used to set up the files in the terminal."""
    print "This is the simplified setup to create learning files."
    print "Default values are used for the variables not asked for."

    # Ask for a file name.
    goal = raw_input("Where should the target color file be stored? (make sure it ends in .pkl) ")

    # Ask the name of the behavior to be used.
    behavior_name = raw_input("WHich behavior is the BBIE file for? ")

    # Get the versions to be used.
    id_list = get_behavior_numbers()

    # Get max duration.
    max_duration = eval(raw_input("What should be the cutoff time in seconds? "))

    # Get top nr.
    top_nr = eval(raw_input("How many of the best options should be considered? "))

    # Next for the variables with a default value.
    # Get use_lower
    use_lower = eval(raw_input("Should the lower boundary be used? (enter True of False, default True) "))

    # Get confidence_level
    confidence_level = eval(raw_input("What should the confidence level be? (enter a percentage from 1 to 100, default 95) "))

    # Get min_to_train.
    min_to_train = eval(raw_input("How many times should each version be tried? (default 5) "))

    # Get max_tries
    max_tries = eval(raw_input("After how many time-outs/failures should the system stop trying the behavior? (default 10) "))

    # Now, create the file.
    create_ML_file(goal, behavior_name, id_list, max_duration, top_nr, use_lower, confidence_level, min_to_train, max_tries)

if __name__ == "__main__":
    main()
