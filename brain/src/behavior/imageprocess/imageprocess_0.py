'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation


class ImageProcess_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.state = 'start'

        # define list of sub-behavior here
        self.CreateService()

        # Init variables
        self.startService = False

        self.selected_behaviors = [
            ("subService", "self.startService == True")
        ]
        pass

    def implementation_update(self):
        if self.state == 'start':
            self.CreateService()
            self.startService = True
            self.state = 'running'
        elif self.state == 'running' and (self.subService.is_finished() or self.subService.is_failed()):
            self.startService = False
            self.state = 'start'

        pass

    def CreateService(self):
        self.subService = self.ab.subimageprocess({})
