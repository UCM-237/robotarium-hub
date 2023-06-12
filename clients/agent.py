# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import time
import zmq
from typing import Protocol
from threading import Thread

# Configure logs
logging.basicConfig(level=logging.DEBUG)

class Agent:
  '''Pre-declare class Agent'''

class Device(Protocol):

  def __init__(self, agent: Agent) -> None:
    '''The constructor optionally receive a list of listeners'''

  def connect(self) -> None:
    '''Establish a connection with the hardware'''

  def on_data(self, topic: str, message: str) -> None:
    '''Handle incoming data'''


class Agent:

  def __init__(self, device_class: Device, id: str, ip: str, cmd_port: int=5555, data_port:int=5556,
               hub_ip: str='127.0.0.1', hub_cmd_port: int=5555, hub_data_port: int=5556) -> None:
    self.id = id
    self.ip = ip
    self.cmd_port = cmd_port
    self.data_port = data_port
    self.hub_ip = hub_ip
    self.hub_cmd_port = hub_cmd_port
    self.hub_data_port = hub_data_port
    context = zmq.Context()
    self.control = context.socket(zmq.REQ)
    self.data = context.socket(zmq.PUB)
    self.data.bind(f'tcp://*:{data_port}')
    self.hub_data = context.socket(zmq.SUB)
    self.device = device_class(agent=self)
    self.device.connect()
    self.register()

  def _get_data_url(self):
    return f'tcp://{self.ip}:{self.data_port}'

  def _get_cmd_url(self):
    return f'tcp://{self.ip}:{self.cmd_port}'

  def _get_hub_data_url(self):
    return f'tcp://{self.hub_ip}:{self.hub_data_port}'

  def _get_hub_cmd_url(self):
    return f'tcp://{self.hub_ip}:{self.hub_cmd_port}'


  def register(self) -> None:
    '''Register to the hub'''
    print(f'Registering agent')
    self.control.connect(self._get_hub_cmd_url())
    self.control.send_json({
      'operation': 'hello',
      'source_id': self.id,
      'payload': {
        'url' : self._get_data_url()
      },
      'timestamp': 1000*time.time(),
    })
    response = self.control.recv_json()
    self.connected = response['result'] == 'ok'
    Thread(target=self.listen).start()


  def listen(self) -> None:
    '''Receive data from other agents'''
    logging.debug(f'Connecting to hub at {self._get_hub_data_url()}')
    self.hub_data.connect(self._get_hub_data_url())
    logging.debug('Subscribing to data/*')
    self.hub_data.setsockopt(zmq.SUBSCRIBE, b'data/')
    self.hub_data.setsockopt_string(zmq.SUBSCRIBE, f'control/{self.id}')
    while True:
      topic = self.hub_data.recv_string()
      message = self.hub_data.recv_string()
      self.device.on_data(topic, message)

  def send(self, topic: str, data: dict) -> None:
    '''Send data to a topic'''
    self.data.send_string(topic, flags=zmq.SNDMORE)
    self.data.send_json(data)


  def send_measurement(self, data) -> None:
    '''Send a new measurement'''
    self.data.send_string('data', flags=zmq.SNDMORE)
    self.data.send_json({
      'operation': 'measurement',
      'source_id': self.id,
      'payload': data,
      'timestamp': 1000*time.time(),
    })


if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = Agent()
    agent.register()
    logging.info(f'Agent {agent.id} is listening')