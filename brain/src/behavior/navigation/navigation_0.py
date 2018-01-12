'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation


class Navigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.startNavigating = False
        self.goto_movebase = self.ab.GotoMoveBase({'fileLocations': "/brain/data/locations/bigroom.dat"})
        self.goto = self.ab.gotowrapper({})

        self.selected_behaviors = [ \
            ("goto_movebase", "True"), \
            ("goto", "self.startNavigating == True"), \
            ]

        self.state = 'enter'
        pass

    def implementation_update(self):

        if self.state == 'enter':
            self.set_goal('kitchen')
            self.startNavigating = True
            self.state = 'goto_kitchen'

        elif self.state == 'goto_kitchen' and (self.goto.is_finished() or self.goto.is_failed()):
            self.state = 'kitchen_visited'
            self.startNavigating = False

        elif self.state == 'kitchen_visited':
            self.state = 'goto_working_room'
            self.set_goal('workingroom')
            self.startNavigating = True

        elif self.state == 'goto_working_room' and (self.goto.is_finished() or self.goto.is_failed()):
            self.state = 'working_room_visited'
            self.startNavigating = False

        elif self.state == 'working_room_visited':
            self.state = 'goto_living_room'
            self.set_goal('livingroom')
            self.startNavigating = True

        elif self.state == 'goto_living_room' and (self.goto.is_finished() or self.goto.is_failed()):
            self.state = 'all_rooms_visited'
            self.startNavigating = False

        pass

    def set_goal(self, goal):
        self.goto = self.ab.gotowrapper({'goal': goal, 'error_range': -1})
