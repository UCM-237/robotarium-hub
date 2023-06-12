# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import threading
import time
import zmq
import json

# Configure logs
logging.basicConfig(level=logging.INFO)

CMD_PORT = 5555
DATA_PORT = 5556


class Server:
  def __init__(self):
    self.OPS = {
      'hello': self.add_agent
    }
    self.context = zmq.Context()
    self.commands_socket = self.context.socket(zmq.PAIR)
    self.commands_socket.bind(f'tcp://*:{CMD_PORT}')
    self.data_socket = self.context.socket(zmq.PUB)
    self.data_socket.bind(f'tcp://*:{DATA_PORT}')
    self.agents = {}
    self.running = True
    self.accepting = threading.Thread(target=self.accept, args=())
    self.accepting.start()

  def add_agent(self, id, url):
    logging.info(f'Agent {id} registered with url {url}');
    self.agents[id] = {
      'url': url,
      'socket': self.context.socket(zmq.SUB)
    }
    self.agents[id]['socket'].connect(url)
    self.agents[id]['socket'].subscribe('data')
    self.agents[id]['thread'] = threading.Thread(target=self.listen, args=(id,))
    self.agents[id]['thread'].start()

  def listen(self, id):
    logging.info(f'Listening to agent {id}')
    while True:
      topic = self.agents[id]['socket'].recv_string()
      message = self.agents[id]['socket'].recv_json()
      logging.info(f'Incoming data from {message["source_id"]} with topic "{topic}"')
      logging.debug(message)
      data = json.loads(message['payload'])
      for k in data:
        topic = f'{topic}/{message["source_id"]}/{k}'
        self.data_socket.send_string(topic, flags=zmq.SNDMORE)
        self.data_socket.send_string(str(data[k]))
      

  def accept(self):
    #  Wait for next request from client
    while self.running:
      logging.info("Robotarium is waiting for new agents")
      message = self.commands_socket.recv_json()
      logging.info(f'Received request from {message["source_id"]}')
      self.OPS[message['operation']](message['source_id'], message['payload']['url'])
      self.commands_socket.send_json({ 'result': 'ok' })



if __name__ == "__main__":
    end = False
    # Wait for commands
    server = Server()
    while not end:
      time.sleep(5)
      logging.info('Command sent')
      server.commands_socket.send_json({ 'operation':'MOVE', 'payload':{ 'v_left':1, 'v_right': 1} })
      time.sleep(5)
      logging.info('Command sent')
      server.commands_socket.send_json({ 'operation':'MOVE', 'payload':{ 'v_left':0, 'v_right': 0} })
      time.sleep(5)
      logging.info('Command sent')
      server.commands_socket.send_json({ 'operation':'MOVE', 'payload':{ 'v_left':-1, 'v_right': -1} })
      time.sleep(5)
      logging.info('Command sent')
      server.commands_socket.send_json({ 'operation':'MOVE', 'payload':{ 'v_left':0, 'v_right': 0} })
