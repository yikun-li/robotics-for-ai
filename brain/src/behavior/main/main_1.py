'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''

import basebehavior.behaviorimplementation


class Main_x(basebehavior.behaviorimplementation.BehaviorImplementation):
    '''this is a behavior implementation template'''

    # this implementation should not define an __init__ !!!

    def implementation_init(self):

        self.state = 'None'

        # define list of sub-behavior here
        self.CreateExample2(None)
        self.CreateExample3()
        self.CreateExample4()

        # Init variables
        self.startExample2 = False
        self.startExample3 = False
        self.startExample4 = False

        self.selected_behaviors = [
            ("subPrintName", "self.startExample2 == True"),
            ("subTimer", "self.startExample3 == True"),
            ("subProbability", "self.startExample4 == True")
        ]

        self.startExample3 = True
        self.count = 1
        pass

    def implementation_update(self):

        if self.count > 5 and self.state == 'None':
            return

        if self.state == 'None':
            self.CreateExample2(None)
            self.startExample2 = True
            self.state = 'Wait'
            self.count += 1

        elif self.state == 'Wait' and self.subPrintName.is_finished():
            self.CreateExample3()
            self.startExample3 = True
            self.state = 'Probability'

        elif self.state == 'Probability' and self.subTimer.is_finished():
            self.CreateExample4()
            self.startExample4 = True
            self.state = 'Alice'

        elif self.state == 'Alice' and self.subProbability.is_failed():
            self.CreateExample4()
            self.startExample4 = True

        elif self.state == 'Alice' and self.subProbability.is_finished():
            self.CreateExample2('Alice')
            self.startExample2 = True
            self.state = 'Final'

        elif self.state == 'Final' and self.subPrintName.is_finished():
            self.CreateExample4()
            self.startExample4 = True
            self.state = 'Probability2'

        elif self.state == 'Probability2' and self.subProbability.is_failed():
            self.CreateExample4()
            self.startExample4 = True

        elif self.state == 'Probability2' and self.subProbability.is_finished():
            self.state = 'None'
            self.set_finished()

    def CreateExample2(self, name):
        self.subPrintName = self.ab.subprintname({'name': name})

    def CreateExample3(self):
        self.subTimer = self.ab.subtimer({})

    def CreateExample4(self):
        self.subProbability = self.ab.subprobability({})
