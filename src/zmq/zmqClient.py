import zmq, sys
import time
from zmq_utils import get_ip_address

if len(sys.argv) == 2:
    ip = sys.argv[1]
    port = sys.argv[2]
else:
    port = "5557"
    ip = get_ip_address()

context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://%s:%s" % (ip, port))

while True:
    msg = socket.recv()
    print msg
    time.sleep(1)
context.destroy()
