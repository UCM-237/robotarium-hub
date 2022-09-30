# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging
import time
import re
import socket
import zmq

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 2
AGENT_NAME = 'Orange'
#AGENT_IP = socket.gethostbyname(socket.gethostname())
AGENT_IP = '147.96.22.221'
AGENT_PORT = 5556

SERVER_IP = '147.96.22.201'

# Init sockets
context = zmq.Context()


class Arduino:
  '''Encapsulates the communication with Arduino'''
  connected: bool = False
  arduino: Serial = None
  thread: Thread = None
  listeners: list = None

  def __init__(self, listeners: list) -> None:
    self.listeners = listeners

  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    ports = list_ports.comports()
#    for p in ports:
#      try:
#        if re.match('ttyUSB.', p.name) is None:
#          logging.info(f'Connecting to {p.device}')
    self.arduino = serial_for_url('/dev/ttyUSB0', baudrate=9600, timeout=5, write_timeout=5)
    self.thread = Thread(target=self.update).start()
    self.connected = True
#          break
#      except:
#        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot connect to arduino, stopping server.')

  def update(self) -> None:
    print('Starting Arduino update thread')
    while True:
      try:
        data = self.arduino.read_until(size=20)
        v_left, v_right = array.array('d', data[0:16])
        for l in self.listeners:
          l.send_measurement({
            'V_left': v_left,
            'V_right': v_right
          })
      except (ValueError, TypeError) as e:
        print('[WARNING] Ignoring invalid data from Arduino')
      except SerialException as e:
        print('[WARNING] Disconnected from Arduino.')
    print('[INFO] Stopping arduino thread')


class Agent:
  proto: str = 'tcp'
  ip: str = AGENT_IP
  id: str = AGENT_ID
  name: str = AGENT_NAME
  port: int = AGENT_PORT

  def __init__(self) -> None:
    context = zmq.Context()
    self.control = context.socket(zmq.PAIR)
    self.data = context.socket(zmq.PUB)
    self.data.bind("tcp://*:5556")

  def register(self, server_url: str) -> None:
    '''Register to the control hub'''
    print(f'Registering agent')
    with self.control.connect(server_url) as s:
      s.send_json({
        'operation': 'hello',
        'source_id': self.id,
        'payload': {
          'url' : f'{self.proto}://{self.ip}:{self.port}'
        },
        'timestamp': 1000*time.time(),
      })
      response = s.recv_json()
      print(response)
      self.connected = response['result'] == 'ok'

  def send_measurement(self, data) -> None:
    '''Sends a new measurement'''
    self.data.send_string('data', flags=zmq.SNDMORE)
    self.data.send_json({
      'operation': 'measurement',
      'source_id': AGENT_ID,
      'payload': data,
      'timestamp': 1000*time.time(),
    })

  def accept_command(self) -> None:
    logging.info('Agent waiting for commands')
    self.control.recv_json()
    logging.info('Command received')

if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = Agent()
    agent.register(f'tcp://{SERVER_IP}:5555')
    Arduino(listeners=[agent]).connect()
    logging.info(f'Agent {agent.name} is listening')
    while not end:
      #  Wait for next request from client
      message = agent.accept_command()
      print(message)
      logging.info(f'Receiving command from server')
    #  logging.info(f'Sending data from agent {agent.name}')
    #  agent.send_measurement()
    #  time.sleep(2)
