import zmq
import random
import sys
import time

port = "5557"
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://10.0.0.9:%s" % port)

while True:
    msg = socket.recv()
    print msg
    time.sleep(1)
