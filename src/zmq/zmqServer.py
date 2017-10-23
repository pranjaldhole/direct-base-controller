import zmq
import random
import sys
import time

port = "5557"
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://10.0.0.10:%s" % port)

while True:
    socket.send("Pranjal to Aaqib")
    msg = socket.recv()
    print msg
    time.sleep(1)
