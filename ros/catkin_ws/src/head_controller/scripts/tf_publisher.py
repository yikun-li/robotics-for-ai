import rospy

from dynamixel_msgs.msg import JointState as JS
from sensor_msgs.msg import JointState
import message_filters
import math


class Controller(object):

    def __init__(self):
        rospy.Subscriber("/pan_controller/state", JS, self.callbackPan)
        rospy.Subscriber("/tilt_controller/state", JS, self.callbackTilt)
        self.pubPan = rospy.Publisher('/panJointState', JointState, queue_size=1)
        self.pubTilt = rospy.Publisher('/tiltJointState', JointState, queue_size=1)
        self.prev_tilt = None;
        
        
    def callbackPan(self, data):
        js = JointState()
        js.header.stamp = rospy.Time.now();
        js.name = ["head_yaw_joint"]
        js.position = [data.current_pos]
        
        self.pubPan.publish(js)
        
    def callbackTilt(self, data):
        js = JointState()
        js.header.stamp = rospy.Time.now();
        js.name = ["head_pitch_joint"]
        js.position = [data.current_pos]
        
        if self.prev_tilt != None:
            if abs(math.degrees(data.current_pos - self.prev_tilt)) < 5:
                self.pubTilt.publish(js);
        
        self.prev_tilt = data.current_pos;

    #pubTilt.publish(js)
'''
def callback(panData, tiltData):
    print panData.current_pos
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ["pan", "tilt"]
    js.position = [panData.current_pos, tiltData.current_pos]
    
    pub.publish(js)    
'''
if __name__ == '__main__':
    rospy.init_node('tf_pantilt_listener')    
    
 #   pan_sub = message_filters.Subscriber('/pan_controller/state', JS)
 #   tilt_sub = message_filters.Subscriber('tilt_controller/state', JS)
 #   ts = message_filters.TimeSynchronizer([pan_sub, tilt_sub],  0.05)
 #   ts.registerCallback(callback) 
    #rospy.Subscriber("/pan_controller/state", JS, callbackPan)
    #rospy.Subscriber("/tilt_controller/state", JS, callbackTilt)
    #pubPan = rospy.Publisher('/panJointState', JointState, queue_size=1)
    #pubTilt = rospy.Publisher('/tiltJointState', JointState, queue_size=1)
    controller = Controller();
    rospy.spin()
