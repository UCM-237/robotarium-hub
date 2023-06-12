# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging
import struct
import json

from agent import Agent

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'Robot_2'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5561
AGENT_DATA_PORT = 5562
# Where the server is 
HUB_IP = '127.0.0.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556


class Robot:
  '''Encapsulates the communication with Arduino'''
  OP_MOVE_WHEEL: int = 2
  OP_VEL_ROBOT: int = 5
  connected: bool = False
  arduino: Serial = None
  thread: Thread = None
  listeners: list = None
  auto_discovery: list = ['Arduino', 'USB2.0-Serial']

  def __init__(self, agent: Agent) -> None:
    self.parsers = {
      self.OP_VEL_ROBOT: self.speed,
    }
    self.operations = {
      'MOVE': { 'id': self.OP_MOVE_WHEEL, 'method': self.move_wheels },
    }
    self.agent = agent


  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    ports = list_ports.comports()
    for p in ports:
      try:
        if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        self.arduino = serial_for_url(p.device, baudrate=9600, timeout=5, write_timeout=5)
        self.thread = Thread(target=self.update).start()
        self.connected = True
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot open device.')


  def update(self) -> None:
    '''Parse data received from the robot'''
    logging.info('Starting Arduino update thread')
    while True:
      try:
        id = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        operation = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        len = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        data = self.arduino.read(size=len)
        measurement = self.parse(operation, data)
        logging.debug(f'Message from {id}: op={operation}, {len} bytes received, data={measurement}')
        self.agent.send_measurement(measurement)
      except (ValueError, TypeError) as e:
        logging.debug('Ignoring invalid data from Arduino')
        print(e)
      except SerialException as e:
        logging.info('Disconnected from Arduino.')
        print(e)


  def move_wheels(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    bytes_written = self.arduino.write(
      struct.pack('H', AGENT_ID) +
      struct.pack('H', self.OP_MOVE_WHEEL) +
      struct.pack('H', 16) +
      struct.pack('d', v_left) +
      struct.pack('d', v_right)
    )


  def speed(self, data) -> dict:
    '''Parse speed from binary data'''
    v_left, v_right = array.array('d', data[0:16])
    return {
      'v_left': v_left,
      'v_right': v_right
    }
  

  def parse(self, operation: int, data: bytes) -> dict:
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    self.operations[operation]['method'](**kwargs)


  def on_data(self, topic: str, message: str) -> None:
    if topic.startswith('control/'):
      cmd = topic[len(f'control/{AGENT_ID}')+1:].upper()
      params = json.loads(message)
      self.exec(cmd, **params)


if __name__ == "__main__":
  agent = Agent(
    device_class=Robot,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )