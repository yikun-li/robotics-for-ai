import roslib; roslib.load_manifest('alice_msgs')
import rospy
from alice_msgs.srv import MemorySrv
import cjson
import json
import sys
rospy.init_node('text_speech_input')
service_write = rospy.ServiceProxy('memory', MemorySrv)
rospy.wait_for_service('memory')

s1 = None
s2 = None

if len(sys.argv) < 2:
	print "usage: speech_to_memory.py \"sentence1\" \"sentence2\""
if len(sys.argv) == 2:
	s1 = sys.argv[1]
	s2 = sys.argv[1]
if len(sys.argv) > 2:
	s1 = sys.argv[1]
	s2 = sys.argv[2]

print json.dumps({'message' : s1, '2best' : [s1,s2]})




 
t = rospy.Time.now()
while t < rospy.Time(1):
    t = rospy.Time.now()
print t
try:
    service_write(rospy.Time.now(), "voice_command", json.dumps({'message' : s1, '2best' : [s1,s2],'azimuth' : 0.153}))
    #service_write(rospy.Time.now(), "Julius", cjson.encode(str(words)))
    print "[tm] command sent"
except:
    print "[tm] Write memory not available"
