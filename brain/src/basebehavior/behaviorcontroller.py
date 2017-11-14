
import basebehavior.allbehaviors
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Behavior.BehaviorController').addHandler(util.nullhandler.NullHandler())

class BehaviorFinished(Exception):
    pass

class BehaviorController:
    """
    Behavior controller:
    Every update step, let the starting behavior chooses a new step. The behavior
    might call sub-behaviors
    """
    def __init__(self, behavior):
        """Initialize the behavior controller: store the starting behavior and the robot ip"""
        self.logger = logging.getLogger('Borg.Brain.Behavior.BehaviorController')

        ab = basebehavior.allbehaviors.AllBehaviors()
        
        if (behavior[-2:] == '})'):
            command = "self.set_behavior(ab." + behavior + ")"
        else: 
            command = "self.set_behavior(ab." + behavior + "({}))"
        
        exec(command)

    def set_behavior(self, behavior):
        self.__behavior = behavior

    def get_behavior(self):
        return self.__behavior

    def update(self):
        """let the behavior choose a next step"""
        b = self.get_behavior()
        if (b.is_stopped()):
            raise BehaviorFinished()

        self.get_behavior().update()


