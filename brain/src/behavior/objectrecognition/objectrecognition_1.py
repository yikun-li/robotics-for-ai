import basebehavior.behaviorimplementation


class ObjectRecognition_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.startSub = False
        self.state = 'running'

        self.CreateTask(0)
        self.selected_behaviors = [("sub", "self.startSub == True")]

        self.state = 'None'
        pass

    def implementation_update(self):
        if self.state == 'None':
            self.CreateTask(1)
            self.startSub = True
            self.state = 'Wait'

        elif self.state == 'Wait' and self.sub.is_finished():
            self.CreateTask(1)
            self.startSub = True
            self.state = 'Wait'

        elif self.state == 'Wait' and self.sub.is_failed():
            print("Failed!!!")
            self.CreateTask(1)
            self.startSub = True
            self.state = 'Wait'

    def CreateTask(self, command):
        self.sub = self.ab.subobjectrecognition({'command': command})
