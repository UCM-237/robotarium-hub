# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import time
import zmq
from typing import Protocol
from threading import Thread

# Configure logs
logging.basicConfig(level=logging.DEBUG)


class Device(Protocol):

  def __init__(self, listeners: list) -> None:
    '''The constructor optionally receive a list of listeners'''

  def connect(self) -> None:
    '''Establish a connection with the hardware'''

  def parse(self, operation: int, data: bytes) -> dict:
    '''Parse incoming data'''


  def exec(self, operation: str, **kwargs) -> None:
    '''Execute incoming commands'''



class Agent:

  def __init__(self, id: str, device_class: Device, ip: str, cmd_port: int=5555, data_port:int=5556,
               server_ip: str='127.0.0.1', server_cmd_port: int=5555, server_data_port: int=5556) -> None:
    self.id = id
    self.ip = ip
    self.cmd_port = cmd_port
    self.data_port = data_port
    self.server_ip = server_ip
    self.server_cmd_port = server_cmd_port
    self.server_data_port = server_data_port
    context = zmq.Context()
    self.control = context.socket(zmq.PAIR)
    self.data = context.socket(zmq.PUB)
    self.data.bind(f'tcp://*:{data_port}')
    self.server_data = context.socket(zmq.SUB)
    self.device = device_class(listeners=[self])
    self.device.connect()

  def _get_data_url(self):
    return f'tcp://{self.ip}:{self.data_port}'

  def _get_cmd_url(self):
    return f'tcp://{self.ip}:{self.cmd_port}'

  def _get_server_data_url(self):
    return f'tcp://{self.server_ip}:{self.server_data_port}'

  def _get_server_cmd_url(self):
    return f'tcp://{self.server_ip}:{self.server_cmd_port}'


  def register(self) -> None:
    '''Register to the hub'''
    print(f'Registering agent')
    self.control.connect(self._get_server_cmd_url())
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
    logging.debug('Connecting to hub at {self._get_server_data_url()}')
    self.server_data.connect(self._get_server_data_url())
    logging.debug('Subscribing to data/*')
    self.server_data.subscribe('data/')
    while True:
      topic = self.server_data.recv_string()
      message = self.server_data.recv_string()
      print(f'{topic} {message}')


  def send_measurement(self, data) -> None:
    '''Sends a new measurement'''
    self.data.send_string('data', flags=zmq.SNDMORE)
    self.data.send_json({
      'operation': 'measurement',
      'source_id': self.id,
      'payload': data,
      'timestamp': 1000*time.time(),
    })


  def accept_command(self) -> None:
    '''Wait for commands'''
    logging.info('Agent waiting for commands')
    command = self.control.recv_json()
    self.device.exec(command['operation'], **command['payload'])
    logging.info('Command received')
    return command



if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = Agent()
    agent.register()
    logging.info(f'Agent {agent.id} is listening')
    while not end:
      #  Wait for next request
      message = agent.accept_command()
      logging.info(f'Receiving command from server')
