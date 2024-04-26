# -*- coding: UTF-8 -*-
#!/bin/python3
from serial import Serial, SerialException, serial_for_url
import logging
import math
import time
import json
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from agent import Agent
import threading

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'AlgorithmFuzzy5'
AGENT_IP = '192.168.1.109'
AGENT_CMD_PORT = 5568
AGENT_DATA_PORT = 5569
# Where the server is 
HUB_IP = '192.168.1.109'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556
Position={}


class Algorithm:
  
  state: dict = {}
  
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.Position={}
    self.Meta = '0'
    self.SAMPLETIME=200
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.next = False
    self.angleCorrected= False
    self.count=0
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.PI=math.pi
    #control 1
    self.AngleError = ctrl.Antecedent(np.arange(-self.PI/2, self.PI/2, 0.1), 'AngleError')
    self.AngleError['bigNegative'] = fuzz.trimf(self.AngleError.universe, [-self.PI/2, -self.PI/2,-self.PI/4])
    self.AngleError['negative'] = fuzz.trimf(self.AngleError.universe, [-self.PI/2, -self.PI/4,0.0])
    self.AngleError['zero'] = fuzz.trimf(self.AngleError.universe, [-self.PI/5,0, self.PI/5])
    self.AngleError['positive'] = fuzz.trimf(self.AngleError.universe, [0.0, self.PI/4, self.PI/2])
    self.AngleError['bigPositive'] = fuzz.trimf(self.AngleError.universe, [self.PI/4, self.PI/2, self.PI/2])
    self.W = ctrl.Consequent(np.arange(-1, 1, 0.01), 'W')
    
    # Membership functions for the consequent
    self.W['farLeft'] = fuzz.trimf(self.W.universe, [-1, -1, -0.45])
    self.W['left'] = fuzz.trimf(self.W.universe, [-0.6, -0.3, 0])
    self.W['center'] = fuzz.trimf(self.W.universe, [-0.2, 0, 0.2])
    self.W['right'] = fuzz.trimf(self.W.universe, [0, 0.3, 0.6])
    self.W['farRight'] = fuzz.trimf(self.W.universe, [0.45, 1, 1])

    self.rule1 = ctrl.Rule(self.AngleError['bigNegative'], self.W['farLeft'])
    self.rule2 = ctrl.Rule(self.AngleError['negative'], self.W['left'])
    self.rule3 = ctrl.Rule(self.AngleError['zero'], self.W['center'])
    self.rule4 = ctrl.Rule(self.AngleError['positive'], self.W['right'])
    self.rule5 = ctrl.Rule(self.AngleError['bigPositive'], self.W['farRight'])

    self.W_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3,self.rule4,self.rule5])
    self.heading=ctrl.ControlSystemSimulation(self.W_ctrl)
    
    #control 2
    self.AngleError2 = ctrl.Antecedent(np.arange(-self.PI, self.PI, 0.05), 'AngleError')
    self.AngleError2['bigNegative'] = fuzz.trimf(self.AngleError2.universe, [-self.PI, -self.PI,-self.PI/2])
    self.AngleError2['negative'] = fuzz.trimf(self.AngleError2.universe, [-self.PI, -self.PI/4,0])
    self.AngleError2['zero'] = fuzz.trimf(self.AngleError2.universe, [-0.3,0,0.3,])
    self.AngleError2['positive'] = fuzz.trimf(self.AngleError2.universe, [0, self.PI/4, self.PI])
    self.AngleError2['bigPositive'] = fuzz.trimf(self.AngleError2.universe, [self.PI/2, self.PI, self.PI])
    self.W2 = ctrl.Consequent(np.arange(-5, 5, 0.02), 'W')
    
    # Membership functions for the consequent
    self.W2['farLeft'] = fuzz.trimf(self.W2.universe, [-5, -5, -3])
    self.W2['left'] = fuzz.trimf(self.W2.universe, [-4, -3, -1])
    self.W2['center'] = fuzz.trimf(self.W2.universe, [-3.5, 0, 3.5])
    self.W2['right'] = fuzz.trimf(self.W2.universe, [1, 3, 4])
    self.W2['farRight'] = fuzz.trimf(self.W2.universe, [3, 5, 5])

    self.rule6 = ctrl.Rule(self.AngleError2['bigNegative'], self.W2['farLeft'])
    self.rule7 = ctrl.Rule(self.AngleError2['negative'], self.W2['left'])
    self.rule8 = ctrl.Rule(self.AngleError2['zero'], self.W2['center'])
    self.rule9 = ctrl.Rule(self.AngleError2['positive'], self.W2['right'])
    self.rule10 = ctrl.Rule(self.AngleError2['bigPositive'], self.W2['farRight'])

    self.W_ctrl2 = ctrl.ControlSystem([self.rule6, self.rule7, self.rule8,self.rule9,self.rule10])
    self.heading2=ctrl.ControlSystemSimulation(self.W_ctrl2)
    
    
    #position
    self.positionDataMeta = {}
  def connect(self):
    logging.debug('starting device')
    #self.thread = threading.Thread(target=self.rendevouz('0','RobotAgent1')).start()
    self.thread = threading.Thread(target=self.rendevouz('5','RobotAgent5')).start()
    
  
  def rendevouz(self,agentId,agentName)-> None:
    while self.Meta not in self.Position or agentId not in self.Position:
      time.sleep(5)
    self.orientation(agentId,agentName)
  
  def sendVel(self,angularWheel,agenName):
    self.agent.send('control/'+agenName+'/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
    
  def correctAngle(self,agentId,agentName):
    self.tval_before=time.time()*1000
    #self.agent.send('control/'+agentName+'/move',{'v_left':0,'v_right':0})
    angleError,modulo=self.computeAngleError(agentId)
    w=0.0
    vel=0
    angularWheel = [0.0, 0.0]
    self.count+=1
    
    if(abs(angleError)<0.40):
      
      self.agent.send('control/'+agentName+'/move',{'v_left':0,'v_right':0})   
      self.angleCorrected=True
      print("angle corrected")
      self.agent.send('control/'+agentName+'/move',{'v_left':0,'v_right':0})
      
    else:
      self.angleCorrected=False
      
      self.heading2.input['AngleError']=angleError
      self.heading2.compute()
      w=float(self.heading2.output['W'])
     
    if abs(w)<3.3:
      if w>0:
        w=3.3
      else:
        w=-3.3
    velocity_robot=[float(w),vel]
    self.angularWheelSpeed(angularWheel,velocity_robot)
    print(angularWheel)
    self.sendVel(angularWheel,agentName)
    self.tval_after=time.time()*1000
    self.tval_sample=self.tval_after-self.tval_before
    if self.tval_sample < 0:
       print("Error de tiempo")
       print(self.tval_sample)
    elif self.tval_sample > 500:
       print("(movimiento) Tiempo del programa mayor: ", self.tval_sample)
    else:

      print((500 - self.tval_sample) / 1000)
      
      time.sleep((500 - self.tval_sample)/1000)
    # self.agent.send('control/'+agentName+'/move',{'v_left':0,'v_right':0})
    # time.sleep(0.5)
      
  def computeAngleError(self,agentId):
    posdataMeta=json.loads(self.Position[self.Meta])
    posdataAgent=json.loads(self.Position[agentId])
    x=float(posdataMeta['x'])-float(posdataAgent['x'])
    y=float(posdataMeta['y'])-float(posdataAgent['y'])
    angle=math.atan2(y,x)
    angleError=float(posdataAgent['yaw'])-angle
    modulo=math.sqrt((x*x)+(y*y))
    print("modulo:")
    print(modulo)
    
    if angleError > self.PI:
      angleError=angleError-2*self.PI
    elif angleError< (-self.PI):
      angleError=angleError+2*self.PI
    print("angleError:")
    print(angleError)
    return angleError,modulo   
  
         
  def orientation(self,agentId,agentName):
    while(self.angleCorrected == False):   
      self.correctAngle(agentId,agentName)
    
    while(True):
      self.tval_before=time.time()*1000 
      w=0.0
      vel=0
      angularWheel = [0.0, 0.0]
      
      x=float(self.positionDataMeta['x'])-float(self.positionDataAgent['x'])
      y=float(self.positionDataMeta['y'])-float(self.positionDataAgent['y'])
      angle=math.atan2(y,x)
      angleError=float(self.positionDataAgent['yaw'])-angle
      modulo=math.sqrt((x*x)+(y*y))
      print("modulo:")
      print(modulo)
      
      if angleError > self.PI:
        angleError=angleError-2*self.PI
      elif angleError< (-self.PI):
        angleError=angleError+2*self.PI
      print("angleError:")
      print(angleError)
        
      w=0
      vel=0
      #first correct the angle
      
      if abs(angleError)>self.PI/2:
        self.heading2.input['AngleError']=angleError
        self.heading2.compute()
        w=float(self.heading2.output['W'])
      else:
          self.heading.input['AngleError']=angleError
          self.heading.compute()
          w=float(self.heading.output['W'])
          
          vel=0
      if modulo > 0.30:
        self.heading.input['AngleError']=angleError
        self.heading.compute()
        w=float(self.heading.output['W'])
        print(w)
        vel=7.5*3.35
        print("GO----------------------------------------")
      else:
        vel=0
        w=0
        print("STOP----------------------------------------")
      velocity_robot=[float(w),vel]
      self.angularWheelSpeed(angularWheel,velocity_robot)
      print(angularWheel)
      self.agent.send('control/'+agentName+'/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
    # print(angularWheel)
      self.tval_after=time.time()*1000
      self.tval_sample=self.tval_after-self.tval_before
      if self.tval_sample < 0:
        print("Error de tiempo")
        print(self.tval_sample)
      elif self.tval_sample > self.SAMPLETIME:
        print("(movimiento) Tiempo del programa mayor: ", self.tval_sample)
      else:

        print((self.SAMPLETIME - self.tval_sample) / 1000)
        
        time.sleep((self.SAMPLETIME - self.tval_sample) / 1000)


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


  def on_data(self,topic: str, message: str) ->None:

    try:
      _, agent, topic = topic.split('/')
      #print(agent)
      #print(message)
    except:
      print("invalid message")
      return
    # if agent not in self.state:
    #   self.state[agent]={}
    # self.state[agent][topic] = ast.literal_eval(message)
    if topic == "position":
      self.Position[agent]=message
      if agent == self.Meta:
        self.positionDataMeta = json.loads(message)
      elif agent == '5':
        self.positionDataAgent = json.loads(message)
      
    
 
    
      


if __name__ == "__main__":
  agent = Agent(
    device_class=Algorithm,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_ip= HUB_IP,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )
  
  
  
 