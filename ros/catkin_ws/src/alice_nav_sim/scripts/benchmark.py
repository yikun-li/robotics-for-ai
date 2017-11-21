import experiment as ep
import gazebo as gz
import threading
import tf
import rospy
import signal

import time

import rospkg
import cv2

from os.path import *

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for the alice_description package
path = rospack.get_path('alice_description')

#TODO: Make this configurable:
door_sdf_location = join(path,"sdf/door.sdf")
fixed_door_sdf_location = join(path,"sdf/fixed_door_nocontact.sdf")
shouldStop = False
stopLock = threading.Lock()



def signal_handler(signal, frame):
        print "killing all threads...."
        stopLock.acquire()
        shouldStop = True
        stopLock.release()
        print "press enter to stop"
        
signal.signal(signal.SIGINT, signal_handler)
        


def handleInput():
    stop = False
    while not stop:
        stopLock.acquire()
        stop = shouldStop
        stopLock.release()
        inp = raw_input("type which door should open: (1, 2, 3) + enter")
        doorIdx = int(inp)
        openDoor(doorIdx)
        
        
def closeDoor(doorIdx, x, y):
    print "close door ", doorIdx
    gz.delete_model('door_%s' % doorIdx)
    gz.pause_physics()
    gz.insert_sdf(fixed_door_sdf_location, 'door_%d' % doorIdx, x , y, 0) #Move the door to closed position
    gz.unpause_physics()
            
    
    
def openDoor(doorIdx):
    print "opening doors"

    x, y, theta = ep.door_positions['door_%s' % doorIdx]['open']
    gz.delete_model('door_%s' % doorIdx)
    gz.pause_physics()
    gz.insert_sdf(fixed_door_sdf_location, 'door_%d' % doorIdx, x , y, theta) #Move the door to closed position
    
    gz.unpause_physics()

if __name__ == "__main__":
    
    rospy.init_node('btool', anonymous=True)
    #Insert doors with random pose
    ep.insert_doors(['open','open','open'])
    transform = tf.TransformListener();

    first_check = False
    second_check = False
    time_buffer = 60.0
    first_check_time = 0.0
    
    sleep_rate = rospy.Rate(5)
    inputThread = threading.Thread(target = handleInput)
    inputThread.start()
    while not first_check:
        sleep_rate.sleep()
        transform.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(2))
        trans, rot = transform.lookupTransform("map", "base_link", rospy.Time(0))
        #if cv2.waitKey(10) != -1:
            #openDoors()
        for i in range(1,4):
            x, y, theta = ep.door_positions['door_%s' % i]['closed']
            distance = ((trans[0] - x)**2 + (trans[1] - y)**2)**0.5
            
            if distance < 1.5:
                #The robot is closer than 1.5 meter to the door location.
                print "Robot is getting close to a door"
                first_check = True
                closeDoor(i, x, y)
                #Adds a timer check for second door_check
                first_check_time = time.time()
                break
                
    #Empty loop until timer is gone
    while time.time() - first_check_time < time_buffer:
        sleep_rate.sleep()
        
    while not second_check:
        sleep_rate.sleep()
        transform.waitForTransform("/odom", "/base_link", rospy.Time(0), rospy.Duration(2))
        trans, rot = transform.lookupTransform("odom", "base_link", rospy.Time(0))
        
        for i in range(1,4):
            x, y, theta = ep.door_positions['door_%s' % i]['closed']
            distance = ((trans[0] - x)**2 + (trans[1] - y)**2)**0.5
            
            if distance < 1.0:
                #The robot is closer than 1 meter to the door location.
                print "Robot is getting close to a door"
                second_check = True
                #Adds a timer check for second door_check
                break
    

