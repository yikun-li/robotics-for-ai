import time

class IntervalCondition(object):
    def __init__(self, interval=1.0):
        self.__interval = interval
        self.__last_time = 0

    def check(self):
        if time.time() >= self.__last_time + self.__interval:
            self.__last_time = time.time()
            return True
        return False

    def reset(self):
        self.__last_time = time.time()

    def __call__(self):
        return self.check()
