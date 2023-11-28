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

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'Algorithm'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5563
AGENT_DATA_PORT = 5564
# Where the server is 
HUB_IP = '127.0.0.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556
Position={}


class Algorithm:
  
  state: dict = {}
  
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.Position={}
    self.Meta = '1'
    self.SAMPLETIME=250
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.next = False
    self.cumError=0
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.PI=math.pi

    self.AngleError = ctrl.Antecedent(np.arange(-self.PI/2, self.PI/2, 0.1), 'AngleError')
    self.AngleError['bigNegative'] = fuzz.trimf(self.AngleError.universe, [-self.PI/2, -self.PI/2,-self.PI/4])
    self.AngleError['negative'] = fuzz.trimf(self.AngleError.universe, [-self.PI/2, -self.PI/4,-0.65])
    self.AngleError['zero'] = fuzz.trimf(self.AngleError.universe, [-0.65,0,0.65,])
    self.AngleError['positive'] = fuzz.trimf(self.AngleError.universe, [0.65, self.PI/4, self.PI/2])
    self.AngleError['bigPositive'] = fuzz.trimf(self.AngleError.universe, [self.PI/4, self.PI/2, self.PI/2])

    self.W = ctrl.Consequent(np.arange(-self.PI, self.PI, 0.01), 'W')
    
    # Membership functions for the consequent
    self.W['farLeft'] = fuzz.trimf(self.W.universe, [-self.PI, -self.PI, -self.PI/2])
    self.W['left'] = fuzz.trimf(self.W.universe, [-self.PI, -self.PI/2, 0])
    self.W['center'] = fuzz.trimf(self.W.universe, [-self.PI/2, 0, self.PI/2])
    self.W['right'] = fuzz.trimf(self.W.universe, [0, self.PI/2, self.PI])
    self.W['farRight'] = fuzz.trimf(self.W.universe, [self.PI/2, self.PI, self.PI])

    self.rule1 = ctrl.Rule(self.AngleError['bigNegative'], self.W['farLeft'])
    self.rule2 = ctrl.Rule(self.AngleError['negative'], self.W['left'])
    self.rule3 = ctrl.Rule(self.AngleError['zero'], self.W['center'])
    self.rule4 = ctrl.Rule(self.AngleError['positive'], self.W['right'])
    self.rule5 = ctrl.Rule(self.AngleError['bigPositive'], self.W['farRight'])

    self.W_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3,self.rule4,self.rule5])
    self.heading=ctrl.ControlSystemSimulation(self.W_ctrl)
  def test(self):
    while(1):
      self.agent.send('control/2/move',{'v_left':8.0,'v_right':8.0})
      time.sleep(2)
      self.agent.send('control/2/move',{'v_left':-1.0,'v_right':-1.0})
      time.sleep(2)
  def connect(self):
    logging.debug('starting device')
    #self.thread = Thread(target=self.test).start()
    
  
  def rendevouz(self,agent)-> None:
    while self.Meta not in Position or agent not in Position:
      pass
    self.orientation(agent)

  def orientation(self,agent):
    
    vel=0
    angularWheel = [0.0, 0.0]
    
    posdataMeta=json.loads(Position[self.Meta])
    posdataAgent=json.loads(Position[agent])
    x=float(posdataMeta['x'])-float(posdataAgent['x'])
    y=float(posdataMeta['y'])-float(posdataAgent['y'])
    angle=math.atan2(y,x)
    angleError=float(posdataAgent['yaw'])-angle
    modulo=math.sqrt((x*x)+(y*y))
    print("angle:")
    print(angleError)
    if angleError > self.PI:
      angleError=angleError-2*self.PI
    elif angleError< (-self.PI):
      angleError=angleError+2*self.PI

    if modulo > 0.25:
      self.heading.input['AngleError']=angleError
      self.heading.compute()
      w=self.heading.output['W']
      print(w)
      vel=8.2*3.35
    else:
      vel=0
      w=0
    velocity_robot=[float(w),vel]
    self.angularWheelSpeed(angularWheel,velocity_robot)
    print(angularWheel)
    self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
   # print(angularWheel)
    if self.tval_sample < 0:
       print("Error de tiempo")
       print(self.tval_sample)
    elif self.tval_sample > self.SAMPLETIME:
       print("(movimiento) Tiempo del programa mayor: ", self.tval_sample)
    else:

      #print((self.SAMPLETIME - self.tval_sample) / 1000)
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
    if self.Meta in Position and '2' in Position:
      self.tval_before=time.time()*1000 
      self.orientation('2')
    if topic == "position":
      Position[agent]=message
      
    
 
    
      


if __name__ == "__main__":
  # algortihm=Algorithm()
  # w=0
  # velocity_robot=[w,6*3.35]
  # angularWheel=[0,0]
  # algortihm.angularWheelSpeed(angularWheel,velocity_robot)
  # print(angularWheel)
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
  
  
  
 