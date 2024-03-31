# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
from threading import Event
import logging
import struct
import json
import LimitsAlgorithm
import OrientationControl
from agent import Agent
import time
import xml.etree.ElementTree as ET

# Configure logs
logging.basicConfig(level=logging.INFO)

# # Who I am
# AGENT_ID = '2'
# AGENT_IP = '192.168.1.115'
# AGENT_CMD_PORT = 5561
# AGENT_DATA_PORT = 5562
# # Where the server is 
# HUB_IP = '192.168.1.109'
# HUB_CMD_PORT = 5555
# HUB_DATA_PORT = 5556


class Robot:
  '''Encapsulates the communication with Arduino'''
  OP_MOVE_ROBOT = 2
  OP_STOP_ROBOT = 3
  OP_TELEMETRY = 4
  OP_TURN_ROBOT = 5
  OP_SILENCE = 6
  OP_POSITION = 7
  OP_CONF_PID = 8
  OP_CONF_FF = 9
  OP_DONE = 10
  INIT_FLAG = 112
  connected: bool = False
  arduino: Serial = None
  Arduinothread: Thread = None
  ArenaRulesThread: Thread = None
  listeners: list = None
  auto_discovery: list = ['Arduino', 'USB2.0-Serial']
  communicationParameters = {}
  agentParameters = {}
  operations = {}
 

  def __init__(self, agent: Agent) -> None:
    self.parsers = {
      self.OP_TELEMETRY: self.speed
    }
    self.operations = {
      'MOVE': { 'id': self.OP_MOVE_ROBOT, 'method': self.move_robot },
      'STOP': { 'id': self.OP_STOP_ROBOT, 'method': self.stop_robot },
      'TELEMETRY': { 'id': self.OP_TELEMETRY, 'method': self.request_telemetry },
      'TURN': { 'id': self.OP_TURN_ROBOT, 'method': self.turn_robot },
      'SILENCE': { 'id': self.OP_SILENCE, 'method': self.silenceCommunication },
      'POSITION': { 'id': self.OP_POSITION, 'method': self.RequestPosition },
      'PID' : { 'id': self.OP_CONF_PID,  'method' : self.conf_PID },
      'FF' : { 'id': self.OP_CONF_FF,  'method' : self.conf_FF }
      }
    self.LimitsAlgorithm = LimitsAlgorithm.LimitsAlgorithm()
    self.Position={}
    self.ArenaLimitsReceived = False
    self.stopCommand = False
    self.operationFromRobotDone = Event()
    self.IgnoreControlCommunication = False
    self.L = 14.5  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.SAMPLETIME=200
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.AgentName
    
    #self.orientationControl = OrientationControl.orientationControl(self.SAMPLETIME)



    self.agent = agent
  def parseConfigurations(self):
    tree = ET.parse('AgentConfiguration.xml')
    root = tree.getroot()
    AgentParameters = root.find('AgentParameters')
    CommunicationParameters = root.find('CommunicationConfiguration')
    
    for agent in AgentParameters:
      self.agentParameters[agent.tag] = agent.text
      
    for communicationParameter in CommunicationParameters:
      self.communicationParameters[communicationParameter.tag] = communicationParameter.text
    for operation in root.find('RobotOperations'):
      self.operations[operation.tag] = operation.text
    
    self.AgentName = self.agentParameters['AgentName']
    
    
  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    ports = list_ports.comports()
    for p in ports:
      try:
        if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        self.arduino = serial_for_url(p.device, baudrate=9600, timeout=5, write_timeout=5)
        self.Arduinothread = Thread(target=self.update).start()
        self.connected = True
        self.ArenaRulesThread = Thread(target=self.checkArenaRules).start()
        
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot open device.')


  def update(self) -> None:
    '''Parse data received from the robot'''
    logging.info('Starting Arduino update Arduinothread')
    
      
    while True:
      try:
        initFlag = int.from_bytes(self.arduino.read(size=4), byteorder='little')
        if initFlag == self.INIT_FLAG:

          id = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          operation = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          len = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          data = self.arduino.read(size=len)
          if operation == self.OP_DONE:
            self.operationFromRobotDone.set()
          else:
            measurement = self.parse(operation, data)
            logging.debug(f'Message from {id}: op={operation}, {len} bytes received, data={measurement}')
            data={'data','AGENT_ID'}
            self.agent.send_measurement(measurement)
          
      except (ValueError, TypeError) as e:
        
        logging.debug('Ignoring invalid data from Arduino')
        print(e)
      
      except SerialException as e:
      
        logging.info('Disconnected from Arduino.')
        print(e)
  
  
  def checkArenaRules(self):
    
    #take the arenaLimits
    while(self.agentParameters['AgentId'] not in self.Position or self.ArenaLimitsReceived == False):
      self.agent.send("localization/RobotariumData","")
      time.sleep(1)
    #First check the arena limits
    while(True):
      robotData = json.loads(self.position[self.agentParameters['AgentId']])
      self.position.pop(self.agentParameters['AgentId'])
      robotX = float(robotData["x"])
      robotY = float(robotData["y"])
      heading = -float(robotData["yaw"])
      newHeading = self.LimitsAlgorithm.checkLimits(robotX,robotY,heading)
      if newHeading != 0:
        #stop control communications
        self.IgnoreControlCommunication = True
        #stop command of moving the robot
        self.move_robot(0,0)
        self.operationFromRobotDone.wait(timeout=0.3)
        if self.operationFromRobotDone.is_set():
          self.operationFromRobotDone.clear()
        #wait for op_done
        self.turn_robot(newHeading)
        self.operationFromRobotDone.wait()
      else:
        self.IgnoreControlCommunication = False
        
      

          
          
          
  def move_robot(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    lent=16#bytes
    data=( struct.pack('<dd', v_left,v_right))
    self.ArduinoSerialWrite(self.OP_MOVE_ROBOT,lent,data)

  
  def stop_robot(self) -> None:
    '''Stop the robot'''
    lent=0
    data=(struct.pack('i', 0) + struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_STOP_ROBOT,lent,data)
  def request_telemetry(self) -> None:
    '''Request telemetry data'''
    lent=0
    data=(struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_TELEMETRY,lent,data)
    
  def turn_robot(self, angle) -> None:
    '''Turn the robot a given angle'''
    #for easy implementation, the angle can be only 90, 180, 270, 360
    lent=4
    data=(struct.pack('i', angle))
    self.ArduinoSerialWrite(self.OP_TURN_ROBOT,lent,data)
    
    
  def silenceCommunication(self) -> None:
    '''Stop the communication with the robot'''
    lent=0
    data=()
    self.ArduinoSerialWrite(self.OP_SILENCE,lent,data)
  def RequestPosition(self) -> None:
    '''Request the position of the robot'''
    lent=0
    data=()
    self.ArduinoSerialWrite(self.OP_POSITION,lent,data)
  def conf_PID(self,P_right, I_right, D_right, P_left, I_left, D_left) -> None:

    len=48#bytes
    data=(struct.pack('d', P_right) +
      struct.pack('d', I_right) +
      struct.pack('d', D_right) +
      struct.pack('d', P_left) +
      struct.pack('d', I_left) +
      struct.pack('d', D_left))
    
    self.ArduinoSerialWrite(self.OP_CONF_PID,len,data)
  def conf_FF(self,FF_right, FF_left) -> None:
    len=16
    data=(struct.pack('d', FF_right) + struct.pack('d', FF_left))
    self.ArduinoSerialWrite(self.OP_CONF_FF,len,data)
    
  def speed(self, data) -> dict:
    '''Parse speed from binary data'''
    v_left, v_right = array.array('d', data[0:16])
    return {
      'v_left': v_left,
      'v_right': v_right
    }
  def batteryStatus(self, data) -> dict:
    pass
  
  def parse(self, operation: int, data: bytes) -> dict:
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    
    if self.stopCommand == True:
      return
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    self.operations[operation]['method'](**kwargs)


  def on_data(self, topic: str, message: str) -> None:
    print(f'{topic} {message}')
    if topic.startswith('control/') and self.IgnoreControlCommunication == False:
      cmd = topic[len(f'control/{self.AgentName}')+1:].upper()
      params = json.loads(message)
      self.exec(cmd, **params)
    else:
      _, agent, topic = topic.split('/')
      if topic== "position":
        if int(agent)==int(self.AgentName):
          self.Position[agent]= message
      elif topic == "ArenaSize":
        self.ArenaLimitsReceived = True
        self.LimitsAlgorithm.setArenaSize(json.loads(message))
               
       
  def angularWheelSpeed(self, w_wheel, velocity_robot):
    fila = 2
    columna = 2
    aux = 0

    for i in range(2):#numwheels
        w_wheel[i] = 0
    
    for i in range(fila):
        for j in range(columna):
            aux += (self.A[i][j] * velocity_robot[j])
        
        w_wheel[i] = aux

        if w_wheel[i] < 0 and w_wheel[i] > -6:
            w_wheel[i] = 0.0
        elif w_wheel[i] > 0 and w_wheel[i] < 6:
            w_wheel[i] = 0.0
        
        aux = 0    

  def ArduinoSerialWrite(self,operation,len,data):
    #in arduino initFlag is uint8_t, agent_id is uint8_t, operation is uint8_t, len is uint16_t
    
    
    head = struct.pack('iii', self.INIT_FLAG, int(self.agentParameters['AgentId']), operation) + struct.pack('i',len)
    message=head + data
    message += b'\n'
    bytes_written= self.arduino.write(message)
		

if __name__ == "__main__":
  robot = Robot
  robot.parseConfigurations(robot)
  agent = Agent(
    device_class=robot,
    id=robot.agentParameters['AgentName'],
    ip=robot.communicationParameters['AgentIp'],
    cmd_port=robot.communicationParameters['AgentCmdPort'],
    data_port=robot.communicationParameters['AgentDataPort'],
    hub_ip = robot.communicationParameters['HubIp'],
    hub_cmd_port = robot.communicationParameters['HubCmdPort'],
    hub_data_port = robot.communicationParameters['HubDataPort'],
  )
  # agent = Agent(
  #   device_class=robot,
  #   id=AGENT_ID,
  #   ip=AGENT_IP,
  #   cmd_port=AGENT_CMD_PORT,
  #   data_port=AGENT_DATA_PORT,
  #   hub_ip = HUB_IP,
  #   hub_cmd_port=HUB_CMD_PORT,
  #   hub_data_port=HUB_DATA_PORT,
  # )
  #agent.device.on_data('control/RobotAgent1/telemetry', '{}')
  
  