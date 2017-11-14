import time
    
class deadlock():
    """
    This class can help you in checking if a Pioneer movement behavior is in deadlock.
    
    If it is, it will have the pioneer turn a random angle.
    
    Usage:
    
    Create a deadlock instance in the init of the behavior using it. Referred to as self.<deadlock_instance> Optional arguments:
    -max_deadlock_time is the timespan over which the deadlock check happens, in seconds/10.
    -minturn in the minimum angle for the random turn in case of deadlock.
    -maxturn is the maximum angle for the turn.
    
    KEEP IN MIND THAT DEFAULTS ARE OVERWRITTEN FOR FUTURE CALLS IF A METHOD IS CALLED WITH NON-DEFAULT ARGUMENTS.
    
    At the start of the update function:
    
        if self.<deadlock_instance>.deadlock:
            self.<deadlock_instance>.update_deadlock()
            return
    
        if self.<deadlock_instance>.check_deadlock:
            return
    
    """
    
    def __init__(self, max_deadlock_time=30, minturn=-180, maxturn=180):
        """
        Setting up the vars. 
        """
        self.deadlock_list = []     # Store last odometry observations
        self.max_deadlock_time = max_deadlock_time # Epoch interval used to check for deadlock, in update steps. Typically 0.1s per step.
        self.max_deadlock_turn = 3  # Time the robot has to make the turn
        self.deadlock = False
        self.deadlock_time = time.time() # Time since last deadlock resolution
        self.minturn = minturn # Minimum turn angle
        self.maxturn = maxturn # Maximum turn angle
        self.turning = False
            
    def check_deadlock(self):
        """ Check if robot is stuck in a corner. """
        """ Returns True or False """
        """ A returned value of True mens the robot is in a deadlock and the system is trying to resolve it by turning. """
        try:
            odo = self.m.get_last_observation('odometry')[1]
        except Exception as e:
            print e
            return False
        self.deadlock_list.insert(0,odo)
        if len(self.deadlock_list) > self.max_deadlock_time:
            old_odo = self.deadlock_list.pop()
            
            x_dif = abs(odo['x']-old_odo['x'])
            y_dif = abs(odo['y']-old_odo['y'])
            a_dif = abs(odo['angle']-old_odo['angle'])
            
#            print 'x_dif {0}'.format(x_dif)
#            print 'y_dif {0}'.format(y_dif)
#            print 'a_dif {0}'.format(a_dif)
            # check difference between current and old odometry 
            if a_dif < 10 and (x_dif + y_dif) < 20:
                # deadlock
                if not self.turning:
                    self.pioneer.turn(random.randint(self.minturn, self.maxturn))
                    print 'INFO: deadlock, turning!'
                    self.turning = True
                    self.deadlock_time = time.time()
                return True
        return False 

    def update_deadlock(self):
        if time.time()-self.deadlock_time > self.max_deadlock_turn:
            self.deadlock = False
