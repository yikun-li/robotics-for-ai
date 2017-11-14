import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.TestSocket').addHandler(util.nullhandler.NullHandler())

class TestSocket(object):
    """
    Dummy to be used instead of sendsocket to test classes.
    Stores the data rather than sending it through TCP/IP
    If the print parameter is true, also prints the data for easy verification
    """

    def __init__(self, to_print=False):
        self.logger = logging.getLogger('Borg.Brain.Util.TestSocket')
        self.data = None
        self.to_print = to_print

    def send(self, data):
        """send a serialized python object through the socket"""
        self.logger.debug(data)
        self.data = data

    def get_data(self):
        """
        Get the last written data. Allows test to check if correct data is
        written
        """
        return data

    def read(self, do_pickle=True):
        """Return the data that was set before"""
        return self.data

    def set_data(self, data):
        """
        Set the  data to be returned on the next read action
        """
        self.data = data
