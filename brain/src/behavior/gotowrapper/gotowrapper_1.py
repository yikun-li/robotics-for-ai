import basebehavior.behaviorimplementation
import time

class GotoWrapper_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    ''' Wrapper for goto using movebase. Depends on goto_movebase behavior '''

    def implementation_init(self):

        self.has_failed = False

        # Check if goal was given, if not: quit
        if not hasattr(self, "goal"):
            self.has_failed = True
            self.set_failed("No goal given")
            return
            
        # Check if goto_movebase is running
        if not self.body.is_movebase_running():
            # Sleep for one second, and check again
            time.sleep(1)
            # If still not running, set failed
            if not self.body.is_movebase_running():
                print "Goto_Movebase is not running"
                self.has_failed = True
                self.set_failed("Goto_MoveBase is not running")
                return
        
        self.goto_dict = {'new_goal': self.goal}
        
        # Check for custom error range
        if hasattr(self, "error_range"):
            self.goto_dict['error_range'] = self.error_range
        
        # Check if the robot should align to goal
        if hasattr(self, "align_to_goal"):
            self.goto_dict['align_to_goal'] = self.align_to_goal
            
        # Check if the robot should align to a certain point, must be a dict containing x and y
        if hasattr(self, "align_to_point"):
            self.goto_dict['align_to_point']= self.align_to_point
            self.goto_dict['align_to_goal'] = False
        
        # Check type of goal
        self.goal_is_a_goalname = False
        if isinstance(self.goal, basestring):
            self.goal_is_a_goalname = True
            
        # State flags
        self.goal_given = False


    def implementation_update(self):
        
        if self.has_failed:
            return

        # Check if we need to give the goal to goto_movebase
        if not self.goal_given:
            self.m.add_item('goto', time.time(), self.goto_dict)
            self.goal_given = True
            return

        # Check if goal is reached
        if self.m.n_occurs('goto') > 0:
            (recogtime, properties) = self.m.get_last_observation('goto')
            if 'status' in properties:
                # Check if we obtained the information from the current goal
                if (self.goal_is_a_goalname and properties['goal_name'] == self.goal) or (not self.goal_is_a_goalname and properties['goal'] == self.goal):
                    if properties['status'] != "goal_reached":
                        # Can't reach goal
                        self.set_failed("Can't reach goal")
                    else:
                        # Goal reached!
                        self.set_finished()
                
