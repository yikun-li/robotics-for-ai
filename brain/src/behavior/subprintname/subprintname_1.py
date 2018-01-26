'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation
import rospy


class SubPrintName_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        if not hasattr(self, 'name') or self.name == None:
            self.name = 'BORG'


    def implementation_update(self):
        self.m.add_item("NameObject", rospy.Time.now(), 'I am ' + self.name)
        # print '*' * 50
        # print "I am " + self.name
        self.set_finished()
