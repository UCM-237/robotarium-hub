# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging
import math
import time
import json


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
    self.SAMPLETIME=150
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.next = False
    self.cumError=0
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]

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
    
    giro = True
    PI=math.pi
    vel=0
    angularWheel = [0.0, 0.0]
   # while(True):

    
    posdataMeta=json.loads(Position[self.Meta])
    posdataAgent=json.loads(Position[agent])
    x=float(posdataMeta['x'])-float(posdataAgent['x'])
    y=float(posdataMeta['y'])-float(posdataAgent['y'])
    modulo=math.sqrt((x*x)+(y*y))
    angle=math.atan2(y,x)
    angleError=float(posdataAgent['yaw'])-angle
    print("angle:")
    print(angleError)
    if angleError > PI:
      angleError=angleError-2*PI
    elif angleError< (-PI):
      angleError=angleError+2*PI
    print("angle:")
    print(angleError)
    self.cumError= self.cumError+angleError

    if angleError<0 and  self.cumError>0:
      self.cumError=0
        
    elif(angleError>0 and  self.cumError<0):
       self.cumError=0
      
    if( self.cumError>0):

      if( self.cumError>14):
          
           self.cumError=14
            
        
    elif( self.cumError<0):

      if( self.cumError<-14):

         self.cumError=-14

    auxcumError=self.cumError
    if modulo>0.25:        
      if(angleError>0.65 or angleError<-0.65):
              
        w=(0.4*angleError)+0.1*self.cumError#*self.SAMPLETIME/1000 
        print("w:")
        print(w)
      else:
        w=0
        giro=True

      
      vel=0.0
     
    
      if giro:
        self.next=True
      self.tval_after=time.time()*1000
      self.tval_sample = self.tval_after-self.tval_before
      if self.next:
        vel=8.2*3.35
    else:
      w=0
      vel=0
    velocity_robot=[w,vel]
    self.angularWheelSpeed(angularWheel,velocity_robot)
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
      print(agent)
      print(message)
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
  
  
  
 