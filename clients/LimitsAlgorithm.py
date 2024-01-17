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

  def connect(self):
    logging.debug('starting device')
    #self.thread = Thread(target=self.test).start()
    
  

  def direction(heading, robot_pos, rect_coords):
      # Supongamos que tienes el "heading" en grados y las coordenadas del rectángulo
      theta = math.radians(heading)
      dx = math.cos(theta)
      dy = math.sin(theta)

      # Coordenadas del rectángulo
      x1, y1, x2, y2 = rect_coords
      robotX, robotY = robot_pos

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
    if topic == "position":
      self.Position[agent]=message
      
    
 
    
      


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
  
  
  
 