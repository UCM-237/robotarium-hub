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
from agent import Agent
import time
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from linearRegression import LinearRegressionModel
import numpy as np
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


class CalibrateFeedForward:
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
  OP_MOVE_WHEELS = 11
  INIT_FLAG = 112
  connected: bool = False
  arduino: Serial = None
  calibrateThread: Thread = None
  listeners: list = None
  communicationParameters = {}
  agentParameters = {}
  operations = {}
 

  def __init__(self, agent: Agent) -> None:
    self.Position={}
    self.wheelVelocities = {}
    self.linearRegressionDataRight={}
    self.linearRegressionDataLeft={}
    self.ArenaLimitsReceived = False
    self.stopCommand = False
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
    self.Meta = '1'
    self.linearRegressionLeft = LinearRegressionModel()
    self.linearRegressionRight = LinearRegressionModel()
    
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
    logging.debug('starting device')
    self.calibrateThread = Thread(target=self.calibrateFeedForward).start()

  def calibrateFeedForward(self):
    # while self.agentParameters['AgentId'] not in self.Position:
    #   time.sleep(1)
    time.sleep(5)
    self.request_telemetry()
    pwmLeft = 0
    pwmRight = 0
    self.move_wheels(pwmLeft, pwmRight)
    time.sleep(1)
    pwmLeft = 100
    pwmRight = 100
    while True:
      self.tval_before = time.time()
      self.move_wheels(pwmLeft, pwmRight)
      time.sleep(0.8)
      self.tval_after = time.time()
      self.tval_sample = self.tval_after - self.tval_before
      velocityData = json.loads(self.wheelVelocities[self.agentParameters['TargetAgent']])
      self.linearRegressionRight.addData(pwmRight, velocityData["w_right"])
      self.linearRegressionLeft.addData(pwmLeft, velocityData["w_left"])
      

      pwmLeft += 2
      pwmRight += 2
      if pwmLeft > 160 or pwmRight > 160:
        self.move_wheels(0, 0)
        break
      
    self.linearRegressionRight.train()
    self.linearRegressionLeft.train()
    print("Right: ", self.linearRegressionRight.get_line_equation())
    print("Left: ", self.linearRegressionLeft.get_line_equation())
    #save the equation in a file
    with open('rightMotorEquation.txt', 'w') as file:
      file.write(self.linearRegressionRight.get_line_equation())
    with open('leftMotorEquation.txt', 'w') as file:
      file.write(self.linearRegressionLeft.get_line_equation())
    #plot
    plt.scatter(list(self.linearRegressionRight.linearRegressionData.keys()), list(self.linearRegressionRight.linearRegressionData.values()), color='blue', label='Data Points')
    plt.plot(list(self.linearRegressionRight.linearRegressionData.keys()), self.linearRegressionRight.predict(np.array(list(self.linearRegressionRight.linearRegressionData.keys())).reshape(-1, 1)), color='red', label='Regression Line')
    plt.xlabel('PWM')
    plt.ylabel('Angular Velocity')
    plt.title('Right Motor')
    plt.legend()
    plt.grid(True)
    
    #save the plot
    plt.savefig('rightMotorPlot.png')
    plt.show()
    plt.scatter(list(self.linearRegressionLeft.linearRegressionData.keys()), list(self.linearRegressionLeft.linearRegressionData.values()), color='blue', label='Data Points')
    plt.plot(list(self.linearRegressionLeft.linearRegressionData.keys()), self.linearRegressionLeft.predict(np.array(list(self.linearRegressionLeft.linearRegressionData.keys())).reshape(-1, 1)), color='red', label='Regression Line')
    plt.xlabel('PWM')
    plt.ylabel('Angular Velocity')
    plt.title('Left Motor')
    plt.legend()
    plt.grid(True)
  
    #save the plot
    plt.savefig('leftMotorPlot.png')
    plt.show()
 
    
    
           
  def move_robot(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    
  def move_wheels(self,pwm_left, pwm_right) -> None:
    '''Set the wheels' speed setpoint'''
    self.agent.send('control/RobotAgent1/move_wheels', {'pwm_left': pwm_left, 'pwm_right': pwm_right})
  
  def stop_robot(self) -> None:
    '''Stop the robot'''
    
  def request_telemetry(self) -> None:
    '''Request telemetry data'''
    #change RObotAgent1 for the name of the robot
    self.agent.send('control/RobotAgent1/telemetry', {'op': self.OP_TELEMETRY})
   
    
  def turn_robot(self, angle) -> None:
    '''Turn the robot a given angle'''
    #for easy implementation, the angle can be only 90, 180, 270, 360
  
    
  def silenceCommunication(self) -> None:
    '''Stop the communication with the robot'''
  
  def RequestPosition(self) -> None:
    '''Request the position of the robot'''
    
  def conf_PID(self,P_right, I_right, D_right, P_left, I_left, D_left) -> None:

    len=48#bytes
    
   
  def conf_FF(self,FF_right, FF_left) -> None:
    len=16
    
  def speed(self, data) -> dict:
    '''Parse speed from binary data'''
    


  def on_data(self, topic: str, message: str) -> None:
    try:
      _, agent, topic = topic.split('/')
      print(topic)
      print(agent)
      print(message)
    except:
      print("invalid message")
      return
    # if agent not in self.state:
    #   self.state[agent]={}
    # self.state[agent][topic] = ast.literal_eval(message)
    if topic == "position":
      self.Position[agent]=message
    if topic == "telemetry":
      self.wheelVelocities[agent]=message
      
               
       
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

		

if __name__ == "__main__":
  ControlCalibrationAgent = CalibrateFeedForward
  ControlCalibrationAgent.parseConfigurations(ControlCalibrationAgent)
  agent = Agent(
    device_class=ControlCalibrationAgent,
    id=ControlCalibrationAgent.agentParameters['AgentName'],
    ip=ControlCalibrationAgent.communicationParameters['AgentIp'],
    cmd_port=ControlCalibrationAgent.communicationParameters['AgentCmdPort'],
    data_port=ControlCalibrationAgent.communicationParameters['AgentDataPort'],
    hub_ip = ControlCalibrationAgent.communicationParameters['HubIp'],
    hub_cmd_port = ControlCalibrationAgent.communicationParameters['HubCmdPort'],
    hub_data_port = ControlCalibrationAgent.communicationParameters['HubDataPort'],
  )