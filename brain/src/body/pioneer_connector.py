import time
import logging
import random
import string
import util.loggingextra
import util.threadedsocket

class PioneerConnector(object):
    def __init__(self, host, port):
        self.socket = util.threadedsocket.ThreadedSocket(host, port, giveup=-1, use_pickle=False, server=False)
        self.odometry = {"x": 0, "y": 0, "angle": 0, "battery_level": 100}
        self.emergency = False
        self.timestamp = time.time()

    def leftright(self, left, right):
        self.socket.send("leftright %d %d" % (left, right))

    def update(self):
        data = self.socket.get_data()
        if data:
            received = string.join(data)
            value_list = received.split()
            correct_value_list = filter(None, value_list)
            if "EMERGENCY" in correct_value_list:
                self.emergency = True
                return
            
            dict = {}
            dict["x"] = value_list[0]
            dict["y"] = value_list[1]
            dict["angle"] = value_list[2]
            dict["battery_level"] = value_list[3]
            self.odometry = dict
            self.timestamp = time.time()

    def close(self):
        self.socket.close()

if __name__ == "__main__":
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)
    pc = PioneerConnector("localhost", 12345)
    last = round(time.time())
    last_odo = time.time()
    try:
        while pc.socket.running:
            time.sleep(0.01)
            cur = round(time.time())
            if last < cur:
                left = random.randint(0, 1000)
                right = random.randint(0, 1000)
                print " moving %d %d" % (left, right)
                pc.leftright(left, right)
                last = cur

            pc.update()
            if pc.emergency:
                print "Emergency situation"
            else:
                if pc.timestamp > last_odo:
                    odo = pc.odometry
                    last_odo = pc.timestamp
                    print repr(odo)
    finally:
        pc.close()
