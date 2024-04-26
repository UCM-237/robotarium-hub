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
import ast

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'Algorithm'
AGENT_IP = '192.168.1.109'
AGENT_CMD_PORT = 5581
AGENT_DATA_PORT = 5582
# Where the server is 
HUB_IP = '192.168.1.109'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556
Position={}


class PIDController:
  def __init__(self, kp=0.3, ki=0.1, kd=0) -> None:
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.prev_error = 0
    self.integral = 0
    self.max_integral = 20
    self.filter_coeff = 0.2  # Coeficiente del filtro de primer orden
    self.last_derivative=0
    
  def compute(self, error: float,sampleTime) -> float:
    self.integral += error*sampleTime
    
    if error<0 and  self.integral>0:
      
      self.integral=0
    elif(error>0 and  self.integral<0):
      
       self.integral=0
       
    if(self.integral>self.max_integral):
        self.integral=self.max_integral
    elif(self.integral<-self.max_integral):
        self.integral=-self.max_integral
        
    derivative = (error - self.prev_error)/sampleTime
    # Aplicar filtro de primer orden a la parte derivativa
    derivative_filtered = self.filter_coeff * derivative + (1 - self.filter_coeff) * self.last_derivative
    self.prev_error = error
    self.last_derivative = derivative_filtered
    output= self.kp * error + self.ki * self.integral + self.kd * derivative_filtered
    return output

  def reset(self) -> None:
    self.prev_error = 0
    self.integral = 0
    self.last_derivative=0

  def set_kp(self, kp: float) -> None:
    self.kp = kp

  def set_ki(self, ki: float) -> None:
    self.ki = ki

  def set_kd(self, kd: float) -> None:
    self.kd = kd

class rendevouz:
  
  state: dict = {}
  
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.Position={}
    self.Meta = '5'
    self.SAMPLETIME=100
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.next = False
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.PI=math.pi
    
    self.AngleCorrected={'0':False,
                         '1':False}
  def test(self):
    vel=8*3.35
    angularWheel = [0.0, 0.0]
    w=0

    self.agent.send('control/RobotAgent1/telemetry', {'op': 11})
    
    while(1):
      vel=9.5*3.35
      velocity_robot=[w,vel]
      self.angularWheelSpeed(angularWheel,velocity_robot)
      self.agent.send('control/RobotAgent1/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
      time.sleep(4)
      # vel=0
      # velocity_robot=[w,vel]
      # self.angularWheelSpeed(angularWheel,velocity_robot)
      # self.agent.send('control/RobotAgent1/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
      # #self.agent.send('control/RobotAgent1/silence', {'op': 6})
      # time.sleep(5)
      
  def connect(self):
    logging.debug('starting device')
    #self.thread = Thread(target=self.test).start()
    self.thread = Thread(target=self.rendevouz('0','RobotAgent1')).start()
    
  def sendVel(self,angularWheel,agentName):
    self.agent.send('control/'+agentName+'/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
    
  def rendevouz(self,agent,agentName)-> None:
    while self.Meta not in self.Position or agent not in self.Position:
      time.sleep(1)
    self.orientation(agent,agentName)
    
  def ComputeAngleErrorAndDistance(self,agent):
    
    posdataMeta=json.loads(self.Position[self.Meta])
    posdataAgent=json.loads(self.Position[agent])
    x=float(posdataMeta['x'])-float(posdataAgent['x'])
    y=float(posdataMeta['y'])-float(posdataAgent['y'])
    modulo=math.sqrt((x*x)+(y*y))
    angle=math.atan2(y,x)
    angleError=float(posdataAgent['yaw'])-angle
    
    if angleError > self.PI:
    
      angleError=angleError-2*self.PI
    
    elif angleError< (-self.PI):
    
      angleError=angleError+2*self.PI
    
    return angleError,modulo
  
  def correctAngleError(self,agent,agentName,pid):
    tval_before=time.time()*1000 
    #self.agent.send('control/RobotAgent1/move',{'v_left':0,'v_right':0})
    angleError,modulo=self.ComputeAngleErrorAndDistance(agent)
    w=0
    vel=0
    angularWheel = [0.0, 0.0]
    if(abs(angleError)<0.40):
      self.AngleCorrected[agent]=True
      
    else:
      self.AngleCorrected[agent]=False
      w=pid.compute(angleError,300/1000)
      print("w: ",w)
    velocity_robot=[float(w),vel]
    self.angularWheelSpeed(angularWheel,velocity_robot)
    print(angularWheel)
    self.sendVel(angularWheel,agentName)
    tval_after=time.time()*1000
    tval_sample=tval_after-tval_before
    if tval_sample < 0:
       print("Error de tiempo")
       print(tval_sample)
    elif tval_sample > 300:
       print("(movimiento) Tiempo del programa mayor: ", tval_sample)
    else:
      
      print((300 - tval_sample) / 1000)
      
      time.sleep((300 - tval_sample)/1000)
      
  def orientation(self,agent,agentName):
    
    giro = True
    PI=math.pi
    vel=0
    angularWheel = [0.0, 0.0]
    cumError = 0
    pid = PIDController(0.95, 0.35, 0.1)
    pid.reset()
    while(self.AngleCorrected[agent]==False):
      angleCorrected = self.correctAngleError(agent,agentName,pid)
      if angleCorrected:
        break
     
      
    print("Angle Corrected-----------------")  
    pid.reset()
    pid.set_kd(0.06)
    pid.set_ki(0.2)
    pid.set_kp(0.5)
    w=0
    while(True):
      tval_before=time.time()*1000 
      angleError,modulo=self.ComputeAngleErrorAndDistance(agent)
     
      vel=8.2*3.35
      angularWheel = [0.0, 0.0]
      if(modulo<0.35):
        vel=0
        w=0
        print("STOP----------------------------------------")
        velocity_robot=[float(w),vel]
        self.angularWheelSpeed(angularWheel,velocity_robot)
        print(angularWheel)
        self.sendVel(angularWheel,agentName)
        break
      if(abs(angleError)<0.30):
        w=0
        
      else:
        self.AngleCorrected[agent]=False
        w=pid.compute(angleError,300/1000)
        print("w: ",w)
      velocity_robot=[float(w),vel]
      self.angularWheelSpeed(angularWheel,velocity_robot)
      print(angularWheel)
      self.sendVel(angularWheel,agentName)
      tval_after=time.time()*1000
      tval_sample=tval_after-tval_before
      if tval_sample < 0:
        print("Error de tiempo")
        print(tval_sample)
      elif tval_sample > 300:
        print("(movimiento) Tiempo del programa mayor: ", tval_sample)
      else:
        
        print((300 - tval_sample) / 1000)
        
        time.sleep((300 - tval_sample)/1000)
  #   posdataMeta=json.loads(self.Position[self.Meta])
  #   posdataAgent=json.loads(self.Position[agent])
  #   x=float(posdataMeta['x'])-float(posdataAgent['x'])
  #   y=float(posdataMeta['y'])-float(posdataAgent['y'])
    
    
  #   modulo=math.sqrt((x*x)+(y*y))
  #   angle=math.atan2(y,x)
  #   angleError=float(posdataAgent['yaw'])-angle
  #   print("angle:")
  #   print(angleError)
  #   if angleError > PI:
  #     angleError=angleError-2*PI
  #   elif angleError< (-PI):
  #     angleError=angleError+2*PI
  #   print("angle:")
  #   print(angleError)
  #   self.cumError= self.cumError+angleError

  #   if angleError<0 and  self.cumError>0:
  #     self.cumError=0
        
  #   elif(angleError>0 and  self.cumError<0):
  #      self.cumError=0
      
  #   if( self.cumError>0):

  #     if( self.cumError>14):
          
  #          self.cumError=14
            
        
  #   elif( self.cumError<0):

  #     if( self.cumError<-14):

  #        self.cumError=-14

  #   auxcumError=self.cumError
  #   if modulo>0.35:        
      
  #     if(angleError>0.65 or angleError<-0.65):
              
  #       w=(0.35*angleError)+0.1*self.cumError#*self.SAMPLETIME/1000 
  #       print("w:")
  #       print(w)
  #     else:
  #       w=0
  #       giro=True

      
     
    
  #     if giro:
  #       self.next=True
  #     self.tval_after=time.time()*1000
  #     self.tval_sample = self.tval_after-self.tval_before
  #     if self.next:
  #       vel=8.2*3.35
  #   else:
  #     w=0
  #     vel=0
  #     print("STOP----------------------------------------")
  #   velocity_robot=[w,vel]
  #   self.angularWheelSpeed(angularWheel,velocity_robot)
  #   self.agent.send('control/RobotAgent1/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
  #   print(angularWheel)
  #   if self.tval_sample < 0:
  #      print("Error de tiempo")
  #      print(self.tval_sample)
  #   elif self.tval_sample > self.SAMPLETIME:
  #      print("(movimiento) Tiempo del programa mayor: ", self.tval_sample)
  #   else:

  #     #print((self.SAMPLETIME - self.tval_sample) / 1000)
  #     time.sleep((self.SAMPLETIME - self.tval_sample) / 1000)


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
      # print(agent)
      # print(message)
    except:
      print("invalid message")
      return
    # if agent not in self.state:
    #   self.state[agent]={}
    # self.state[agent][topic] = ast.literal_eval(message)

    if topic == "position":
      self.Position[agent]=message
      
    
 
    
      


if __name__ == "__main__":
  agent = Agent(
    device_class=rendevouz,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_ip= HUB_IP,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )
  
  
  
 