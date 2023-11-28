import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://127.0.0.1:4242")

while True:
    #  Wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    #  Do some 'work'
    #time.sleep(1)

    #  Send reply back to client
   
    #print('hi')

    #  Do some 'work'

    #  Send reply back to client
    #socket.send(b"World")
    #print('Response sent')
