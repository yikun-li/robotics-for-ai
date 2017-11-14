from util.threadedsocket import ThreadedSocket

import util.nullhandler
import util.loggingextra
import sys
import logging
import time

if __name__ == '__main__':
    host = "localhost"
    port = 49152
    if len(sys.argv) > 2:
        host = sys.argv[1]
        port = int(sys.argv[2])
    sock = ThreadedSocket(host, port, giveup=2)
    sock.start()
    try:
        start = time.time()
        while not sock.connected and time.time() - start < 1:
            time.sleep(0.01)
        if sock.connected:
            sock.send("diag")
            start = time.time()
            data = []
            while time.time() - start < 2 and data == []:
                time.sleep(0.01)
                data = sock.get_data()
            for item in data:
                if 'status' in item:
                    if item['status'] == 'Communicator is running ok':
                        print "Communicator at %s is running on port %d" % (host, port)
                        sock.close()
                        sys.exit(0)

            print "Communicator at %s is not running on port %d" % (host, port)
            sock.close()
            sys.exit(1)
        else:
            print "Communicator at %s is not running on port %d" % (host, port)
            sys.exit(1)
    finally:
        sock.close()
