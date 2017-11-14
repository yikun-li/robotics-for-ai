import pickle
import csv
import os

"""
This program is used to turn a .pkl file containing a dictionary for BBIE purposes into a .csv file.
"""

def main_alt():
    """This is a program to make .pkl files used in BBIE more readable."""
    print "This is a program to make .pkl files used in BBIE more readable."

    target_file = raw_input("Which file should be read? (give path from current folder) ")

    # Read the data.
    input_file = open(target_file)
    bbie_data = pickle.load(input_file)
    input_file.close()

    # Get an output file
    output_target = raw_input("Where should the simplified file be stored? (give path from current folder, make it .csv) ")

    # Check it's there.
    if not not os.path.exists(output_target):
        # Create it if needed.
        fi = open(output_target,'w')
        fi.close()

    # Try to store the simplified file.
    w = csv.writer(open(str(output_target), "w"))
    for key, val in bbie_data.items():
        w.writerow([key, val])
        if key == "behaviors":
            behaviors = val
            for el in behaviors:
                w.writerow([])
                for key2, val2 in el.items():
                    w.writerow([key2, val2])

            w.writerow([])

if __name__ == '__main__':
    print "Running as main."
    main_alt()

