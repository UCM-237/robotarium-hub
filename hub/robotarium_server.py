# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import threading
import time
import zmq

# Configure logs
logging.basicConfig(level=logging.INFO)

# Init socket
context = zmq.Context()
commands_socket = context.socket(zmq.PAIR)
commands_socket.bind("tcp://*:5555")
agents = {}

def add_agent(id, url):
  logging.info(f'Agent {id} registered with url {url}');
  agents[id] = {
    'url': url,
    'socket': context.socket(zmq.SUB)
  }
  agents[id]['socket'].connect(url)
  agents[id]['socket'].subscribe('data')
  agents[id]['thread'] = threading.Thread(target=listen, args=(id,))
  agents[id]['thread'].start()

def listen(id):
  logging.info(f'Listening to agent {id}')
  while True:
    topic = agents[id]['socket'].recv_string()
    message = agents[id]['socket'].recv_json()
    #logging.info(f'Incoming data from {message["source_id"]} with topic "{topic}"')
    logging.debug(message)

OPS = {
  'hello': lambda id, url: add_agent(id, url)
}


class Server:

  def __init__(self):
    self.running = True
    self.accepting = threading.Thread(target=self.accept, args=()).start()

  def accept(self):
    #  Wait for next request from client
    while self.running:
      logging.info("Robotarium is waiting for new agents")
      message = commands_socket.recv_json()
      logging.info(f'Received request from {message["source_id"]}')
      OPS[message['operation']](message['source_id'], message['payload']['url'])
      commands_socket.send_json({ 'result': 'ok', })
      commands_socket.send_json({ 'operation':'Move', 'payload':{ 'v_left':0.8, 'v_right': 100} })



if __name__ == "__main__":
    end = False
    # Wait for commands
    server = Server()
    #while not end:
    #  logging.info('Command sent')
    #  commands_socket.send_json({ 'operation':'Move', 'payload':{ 'v_left':0.8, 'v_right': 100} })
    #  time.sleep(2)

