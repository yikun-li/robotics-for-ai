
import time
import serial
import brain

import logging
import util.nullhandler

logging_namespace = 'Borg.Brain.Util.Arduino'
logging.getLogger(logging_namespace).addHandler(util.nullhandler.NullHandler())


class Arduino(object):
    """
    Used to get basic sensor information from the Arduino device.
    WARNING:
    - Make sure to add yourself to the dailout group:
    sudo usermod -a -G dialout <username>
    TODO: Fix issues related to hotplugging the device (e.d. after starting this module).
    """

    def __init__(self, port = "/dev/ttyACM0"):
        self.__logger = logging.getLogger(logging_namespace)

        self.__port = port
        self.__serial = None
        self.__pre_settings = None
        self.__retry_timeout = 3.0
        self.__retry_start = time.time()

        self.__topic_dict = {}

        #Used to buffer part of a line (so we are certain to process a complete single line);
        self.__pre_line = ""

        #States: CONNECT, RUN, RETRY
        self.__state = "CONNECT"

    def __connect(self):
        try:
            self.__disconnect()
            #Non-blocking:
            self.__serial = serial.Serial(self.__port, 9600, timeout = 0)
            if self.__pre_settings:
                self.__serial.applySettingsDict(self.__pre_settings)
            self.__serial.open()
            self.__pre_settings = self.__serial.getSettingsDict()
            self.__logger.info("Connected to Arduino on %s" % self.__port)
            return True
        except Exception as e:
            self.__logger.error(e)
            self.__logger.error("Unable to connect, retrying in %s seconds..." % self.__retry_timeout)
            return False

    def __disconnect(self):
        if self.__serial:
            self.__pre_line = ""
            self.__serial.close()

    def __receive(self):
        try:
            line = self.__serial.readline(255)
            #Make sure to process a complete line:
            if line == "":
                return True
            if line[-1] == '\n':
                line = self.__pre_line + line[:-1]
                self.__pre_line = ""
                try:
                    self.__decode_line(line)
                except Exception as e:
                    self.__logger.warning(e)
                    self.__logger.warning("Decoding error... (-> noise or wrong device selected?).")
            else:
                self.__pre_line += line
            return True
        except Exception as e:
            self.__logger.error(e)
            self.__logger.error("Unable to receive data, retrying in %s seconds..." % self.__retry_timeout)
            return False

    def __decode_line(self, line):
        value_string_list = line.split(",")
        for value_string in value_string_list:
            (name, value) = value_string.split("=")
            (name, value) = (name.strip(), value.strip())
            self.__topic_dict[name] = float(value)

    def get_state(self):
        return self.__state

    def is_connected(self):
        return not (self.__state == "CONNECT" or self.__state == "RETRY")

    def get(self, topic):
        if topic in self.__topic_dict:
            return self.__topic_dict[topic]
        else:
            return None

    def update(self):
        """
        Connects, reads and processes data to and from the Arduino.
        To be executed as often as possible (at 10 Hz or so).
        """
        if self.__state == "CONNECT":
            if self.__connect():
                self.__state = "RUN"
            else:
                self.__state = "RETRY"
                self.__retry_start = time.time()
        elif self.__state == "RETRY":
            if (time.time() - self.__retry_start) > self.__retry_timeout:
                self.__state = "CONNECT"
        elif self.__state == "RUN":
            if not self.__receive():
                self.__state = "RETRY"
                self.__retry_start = time.time()

    def __del__(self):
        self.__disconnect()


if __name__ == "__main__":
    brain.setup_logging(logging.getLogger(logging_namespace), None, None, "DEBUG")
    arduino = Arduino()
    while True:
        arduino.update()
        if arduino.is_connected():
            (hum, temp) = (arduino.get("hum"), arduino.get("temp"))
            if hum and temp: 
                print "hum: %f, temp: %f" % (hum, temp)
        time.sleep(0.1)
