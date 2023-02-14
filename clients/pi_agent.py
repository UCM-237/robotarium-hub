# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging
import time
import struct
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


class Robot:
  '''Encapsulates the communication with Arduino'''
  OP_MOVE_WHEEL: int = 2
  OP_VEL_ROBOT: int = 4
  OP_CONF_PID: int = 9
  connected: bool = False
  arduino: Serial = None
  thread: Thread = None
  listeners: list = None
  auto_discovery: list = ['Arduino', 'USB2.0-Serial','Arduino NANO 33 IoT']

  def __init__(self, listeners: list) -> None:
    self.parsers = {
      self.OP_VEL_ROBOT: self.speed,
    }
    self.operations = {
      'MOVE': { 'id': self.OP_MOVE_WHEEL, 'method': self.move_wheels },
      'PID' : { 'id': self.OP_CONF_PID,  'method' : self.K },
    }
    self.listeners = listeners


  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    ports = list_ports.comports()
    for p in ports:
      try:
        if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        self.arduino = serial_for_url(p.device, baudrate=9600, timeout=5, write_timeout=5)
        self.thread = Thread(target=self.update).start()#start a thread that recive the data from arduino
        self.connected = True
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot connect to arduino, stopping server.')


  def update(self) -> None:
    '''Parse data received from the robot'''
    print('Starting Arduino update thread')
    while True:
      try:
        id = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        operation = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        len = int.from_bytes(self.arduino.read(size=2), byteorder='little')
        data = self.arduino.read(size=len)
        measurement = self.parse(operation, data)#comprueba que la operacion existe
        logging.debug(f'Message from {id}: op={operation}, {len} bytes received, data={measurement}')
        
        for l in self.listeners:
          l.send_measurement(measurement)
      except (ValueError, TypeError) as e:
        print('[WARNING] Ignoring invalid data from Arduino')
        print(e)
      except SerialException as e:
        print('[WARNING] Disconnected from Arduino.')
        print(e)
    print('[INFO] Stopping arduino thread')


  def move_wheels(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    bytes_written = self.arduino.write(
      struct.pack('H', AGENT_ID) +
      struct.pack('H', self.OP_MOVE_WHEEL) +
      struct.pack('H', 16) + #size of data
      struct.pack('d', v_left) +
      struct.pack('d', v_right)
    )
    self.arduino.flush()


  def speed(self, data) -> dict:
    '''Parse speed from binary data'''
    v_left, v_right = array.array('d', data[0:16])
    return {
      'v_left': v_left,
      'v_right': v_right
    }
  
  def K(self,P_right, I_right, D_right, P_left, I_left, D_left) -> None:

     bytes_written = self.arduino.write(
      struct.pack('H', 112) + 
      struct.pack('H', AGENT_ID) +
      struct.pack('H', self.OP_CONF_PID) +
      struct.pack('H', 48) + #size of data
      struct.pack('d', P_right) +
      struct.pack('d', I_right) +
      struct.pack('d', D_right) +
      struct.pack('d', P_left) +
      struct.pack('d', I_left) +
      struct.pack('d', D_left) 
    )
     self.arduino.flush()

  def parse(self, operation: int, data: bytes) -> dict: #comprueba que la operación esta en la lista de operaciones
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    self.operations[operation]['method'](**kwargs)


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
    self.robot = Robot(listeners=[self])#le pasa la clase que se ha conectado en un principio
    self.robot.connect()


  def register(self, server_url: str) -> None: #Hello op
    '''Register to the control hub'''
    print(f'Registering agent')
    self.control.connect(server_url)
    self.control.send_json({
      'operation': 'hello',
      'source_id': self.id,
      'payload': {
        'url' : f'{self.proto}://{self.ip}:{self.port}'
      },
      'timestamp': 1000*time.time(),
    })
    response = self.control.recv_json()
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
    command = self.control.recv_json()
    self.robot.exec(command['operation'], **command['payload'])
    logging.info('Command received')
    return command


if __name__ == "__main__":
    end = False
    # Wait for commands
   # robot.update()
    agent = Agent()
    #agent.register(f'tcp://{SERVER_IP}:5555')
    #logging.info(f'Agent {agent.name} is listening')
    while True:

      agent.robot.K(1,1,1,2,2,2)
      time.sleep(5)
      agent.robot.K(2,2,2,1,1,1)
      time.sleep(5)
    '''agent.send_measurement(2)
    while not end:
      #  Wait for next request
      message = agent.accept_command()
      logging.info(f'Receiving command from server')'''
   
