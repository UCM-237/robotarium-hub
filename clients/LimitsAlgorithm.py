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
AGENT_ID = 'LimitsAlgorithm'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5563
AGENT_DATA_PORT = 5564
AGENT_ID_NUM= '0'
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
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.ArenaLimits = {}

  def connect(self):
    logging.debug('starting device')
   
    self.thread = Thread(target=self.Algorithm).start()
    
  
  def Algorithm(self):
     #first of all request the Arena limits
   
    while(1):
      self.direction()
  def direction(self):
     
      while (AGENT_ID_NUM not in self.Position) or len(self.ArenaLimits)==0:
        if(len(self.ArenaLimits)==0):
          self.agent.send("localization/RobotariumData","")
        time.sleep(1)
      position=self.Position
      robotData = json.loads(position[AGENT_ID_NUM])
      position.pop(AGENT_ID_NUM)
      robotX = float(robotData["x"])
      robotY = float(robotData["y"])
      heading = -float(robotData["yaw"])
      dx = math.cos(heading)
      if dx < 0.08 and dx > -0.08:
          dx = 0
      dy = math.sin(heading)
      if dy < 0.08 and dy > -0.08:
          dy = 0

      # coordinates of the rectangle
      x1=self.ArenaLimits["x1"]
      y1=self.ArenaLimits["y1"] 
      x2=self.ArenaLimits["x2"]
      y2=self.ArenaLimits["y2"]
      x3=self.ArenaLimits["x3"]
      y3=self.ArenaLimits["y3"]
      x4=self.ArenaLimits["x4"]
      y4=self.ArenaLimits["y4"]



      # Comprobar la dirección
      if dx > 0:
          if dy > 0:
              print('El robot se está moviendo hacia la esquina superior derecha.')
              # Puedes comparar con la esquina superior derecha del rectángulo
              if x2 > robotX and y2 > robotY:
                  print('El robot se dirigirá hacia la esquina superior derecha del rectángulo.')
          elif dy < 0:
              print('El robot se está moviendo hacia la esquina inferior derecha.')
              # Puedes comparar con la esquina inferior derecha del rectángulo
              if x2 > robotX and y1 < robotY:
                  print('El robot se dirigirá hacia la esquina inferior derecha del rectángulo.')
          else:
              print('El robot se está moviendo hacia la derecha.')
              # Puedes comparar con la pared derecha del rectángulo
              if x2 > robotX:
                  print('El robot se dirigirá hacia la pared derecha del rectángulo.')
      elif dx < 0:
          if dy > 0:
              print('El robot se está moviendo hacia la esquina superior izquierda.')
              # Puedes comparar con la esquina superior izquierda del rectángulo
              if x1 < robotX and y2 > robotY:
                  print('El robot se dirigirá hacia la esquina superior izquierda del rectángulo.')
          elif dy < 0:
              print('El robot se está moviendo hacia la esquina inferior izquierda.')
          else:
              print('El robot se está moviendo hacia la izquierda.')
          
      else:
          if dy > 0:
              print('El robot se está moviendo hacia arriba.')
          elif dy < 0:
              print('El robot se está moviendo hacia abajo.')
          else:
              print('El robot no se está moviendo.')



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
     # print(message)
    except:
      print("invalid message")
      return
    #register the position of the robot in a dictionary
    if topic == "position":
      self.Position[agent]=message
    elif topic == "ArenaSize":
      self.ArenaLimits = json.loads(message)
     # print(self.ArenaLimits)
      
    
 
    
      


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
  
  
  
 