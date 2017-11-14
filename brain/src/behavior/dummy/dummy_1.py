import basebehavior.behaviorimplementation

class Dummy_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        
        self.state = "state1";

    def implementation_update(self):
        
        if self.state == "state1":
            print 'I am state 1';
            self.state = "state2";
            
        elif self.state == "state2":
            print 'I am state 2';
            self.state = "state1";
