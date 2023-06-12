# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import zmq
import json

from time import sleep
from threading import Thread

# Configure logs
logging.basicConfig(level=logging.DEBUG)

CMD_PORT = 5555
DATA_PORT = 5556

class RobotariumHub:
  def __init__(self):
    self.OPS = {
      'hello': self.add_agent
    }
    self.context = zmq.Context()
    self.commands_socket = self.context.socket(zmq.REP)
    self.commands_socket.bind(f'tcp://*:{CMD_PORT}')
    self.data_socket = self.context.socket(zmq.PUB)
    self.data_socket.bind(f'tcp://*:{DATA_PORT}')
    self.agents = {}
    self.running = True
    self.accepting = Thread(target=self.accept, args=())
    self.accepting.start()
    self.poller = zmq.Poller()
    self.listening = Thread(target=self.listen, args=())
    self.listening.start()

  def add_agent(self, id, url):
    logging.info(f'Agent {id} registered with url {url}');
    self.agents[id] = {
      'url': url,
      'socket': self.context.socket(zmq.SUB)
    }
    self.agents[id]['socket'].connect(url)
    self.agents[id]['socket'].subscribe('')
    self.poller.register(self.agents[id]['socket'], zmq.POLLIN)

  def listen(self):
    logging.info(f'Listening to agents')
    while self.running:
      sleep(0.01)
      socks = dict(self.poller.poll())
      for a in self.agents:
        s = self.agents[a]['socket']
        if s in socks and socks[s] == zmq.POLLIN:
          try:
            topic = s.recv_string()
            message = s.recv_json()
          except json.JSONDecodeError:
            logging.error(f'Agent {id} sent invalid JSON')

          if topic == 'data':
            data = message['payload']
            for k in data:
              topic = f'{topic}/{message["source_id"]}/{k}'
            self.data_socket.send_string(topic, flags=zmq.SNDMORE)
            self.data_socket.send_json(data[k])
          else:
            self.data_socket.send_string(topic, flags=zmq.SNDMORE)
            self.data_socket.send_json(message)

  def accept(self):
    '''Wait for next request from client'''
    while self.running:
      logging.info("Robotarium is waiting for new agents")
      message = self.commands_socket.recv_json()
      logging.info(f'Received request from {message["source_id"]}')
      self.OPS[message['operation']](message['source_id'], message['payload']['url'])
      self.commands_socket.send_json({ 'result': 'ok' })



if __name__ == "__main__":
    hub = RobotariumHub()