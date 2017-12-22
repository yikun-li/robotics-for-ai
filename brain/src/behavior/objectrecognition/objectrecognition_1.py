import os
import sys

import basebehavior.behaviorimplementation


class ObjectRecognition_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.chooseNeuralNetwork = 'Google'
        self.NeuralNetwork = {'Our': 1, 'Google': 2}
        self.blockPrint()
        self.startSub = False
        self.state = 'running'

        self.CreateTask(0)
        self.selected_behaviors = [("sub", "self.startSub == True")]

        self.state = 'None'
        pass

    def implementation_update(self):
        if self.state == 'None':
            self.CreateTask(self.NeuralNetwork[self.chooseNeuralNetwork])
            self.startSub = True
            self.state = 'Wait'

        elif self.state == 'Wait' and self.sub.is_finished():
            self.CreateTask(self.NeuralNetwork[self.chooseNeuralNetwork])
            self.startSub = True
            self.state = 'Wait'

        elif self.state == 'Wait' and self.sub.is_failed():
            self.CreateTask(self.NeuralNetwork[self.chooseNeuralNetwork])
            self.startSub = True
            self.state = 'Wait'

    def CreateTask(self, command):
        self.sub = self.ab.subobjectrecognition({'command': command})

    # Disable
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    # Restore
    def enablePrint(self):
        sys.stdout = sys.__stdout__
