

'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation
import rospy

class SubTimer_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    '''this is a behavior implementation template'''
    def implementation_init(self):

	self.timeNow = rospy.Time.now() 
        pass

    def implementation_update(self):

	if (rospy.Time.now() - self.timeNow > rospy.Duration(2)):
	    self.set_finished()
        pass



