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
import numpy as np

from agent import Agent

from OrientationControl import orientationControl

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'LimitsAlgorithm'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5563
AGENT_DATA_PORT = 5564
AGENT_ID_NUM= '2'
# Where the server is 
HUB_IP = '127.0.0.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556
Position={}

class segment:
   def __init__(self,pi,pf):
     self.pi=np.array(pi)
     self.pf=np.array(pf)

class Algorithm:
  
  state: dict = {}
  
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.Position={}
    self.PostionForControl={}
    self.L = 12.4  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.ArenaLimits = {}
    self.OrientationControl = orientationControl()
    self.orientationCorrected = False
    self.newHeadingRequired = False
    self.newHeading=0
    self.SAMPLETIME=200
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0

  def connect(self):
    logging.debug('starting device')
   
    self.thread = Thread(target=self.Algorithm).start()
    self.thread = Thread(target=self.correctOrientation).start()
    
  
  def Algorithm(self):
     #first of all request the Arena limits
   
    while(1):
      self.direction()

  def direction(self):
    minimumSafetyDistance=10
    w=0
    vel=0
    angularWheel = [0.0, 0.0]
    
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
    self.newHeading = 0
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

    segmentOfTheRectangle =[                                  #S1       
        segment((x1,y1),(x2,y2)),#segment 1              -------------------
        segment((x2,y2),(x3,y3)),#segment 2              |                 |         
        segment((x3,y3),(x4,y4)),#segment 3          s4  |                 | #s2
        segment((x4,y4),(x1,y1))  #segment 4             |                 |
                                                        #------------------- 
    ]                                                       #s3
    d=0 #distance to the nearest wall
    # Comprobar la dirección
    if self.newHeadingRequired == False:
        if dx > 0:
            if dy > 0:
                print('The robot is moving to the upper righ')
                # compute the nearest point to the segment, always compare two segment becouse the noise of the robot
                if x2 > robotX and y2 > robotY:
                    print('The robot will move to the upper right corner of the rectangle.')
                    # compute the nearest point to the segment, always compare two segment becouse the noise of the robot
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[1])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading=-heading
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                            
                else:
                    #stop the robot
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the left")          
            elif dy < 0:
                print('The robot is moving to the lower right corner.')
                if x2 > robotX and y1 < robotY:
                    print('The robot will move to the lower right corner of the rectangle.')
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading   
                else:
                    #stop the robot
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the right")
            else:
                print('The robot is moving to the right.')
                if x2 > robotX:
                    print('The robot will move to the right wall of the rectangle.')
                    #in this case compare with 3 segments
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[1])
                    d3 = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -heading
                        elif d == d2:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading
        elif dx < 0:
            if dy > 0:
                print('The robot is moving to the upper left corner.')
                if x1 < robotX and y2 > robotY:
                    print('The robot will move to the upper left corner of the rectangle.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -heading
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left.TODO: STOP THE ROBOT and turn it to the right")
            elif dy < 0:
                print('The robot is moving to the lower left corner.')
                if x1 < robotX and y1 < robotY:
                    print('El robot se dirigirá hacia la esquina inferior izquierda del rectángulo.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[1])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO: STOP THE ROBOT and turn it to the right")
            else:
                print('The robot is moving to the left.')
                #compare with the left wall of the rectangle and the three segments 
                if x1 < robotX:
                    print('The robot will move to the left wall of the rectangle.')
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[1])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                    d3 = self.distance(robotX, robotY, segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        elif d == d2:
                            self.newHeading = -heading
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO : STOP THE ROBOT and turn it to the right")  
        else:
            if dy > 0:
                print('The robot is moving up.')
                d = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                if d<minimumSafetyDistance:
                    self.newHeading = -heading
            elif dy < 0:
                print('The robot is moving down.')
                d = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                if d<minimumSafetyDistance:
                    self.newHeading = -(math.pi - abs(heading))
            else:
                print('El robot no se está moviendo, O no se detecta')
        
        if self.newHeading != 0:
            #stop robot
            w=0
            vel=0
            velocity_robot=[w,vel]
            self.angularWheelSpeed(angularWheel,velocity_robot)
            self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
            time.sleep(1)
            self.self.newHeadingRequired=True
        
            
            
  def correctOrientation(self):
    vel=0
    angularWheel = [0.0, 0.0]
    uniformVelocity = False
    
    while True:
        if len(self.PostionForControl)>0:
            
            robotData = json.loads(self.PostionForControl[AGENT_ID_NUM])
            self.PostionForControl.pop(AGENT_ID_NUM)
            robotX = float(robotData["x"])
            robotY = float(robotData["y"])
            heading = -float(robotData["yaw"])
        #now correct orientation 
            if self.newHeadingRequired == True:
                uniformVelocity=False
                w =  orientationControl.PID(heading, self.newHeading)
                if w !=0:
                    velocity_robot=[w,vel]
                    self.angularWheelSpeed(angularWheel,velocity_robot)
                    self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
                else :
                    self.newHeadingRequired = False
                    self.Pos
            elif(uniformVelocity==False):
                vel=8.2*3.35
                velocity_robot=[w,vel]
                self.angularWheelSpeed(angularWheel,velocity_robot)
                self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
                uniformVelocity=True
        self.tval_after=time.time()*1000
        self.tval_sample = self.tval_after-self.tval_before
        if self.tval_sample < 0:
            print("Error de tiempo")
            print(self.tval_sample)
        elif self.tval_sample > self.SAMPLETIME:
            print("(movimiento) Tiempo del programa mayor: ", self.tval_sample)
        else:
            #print((self.SAMPLETIME - self.tval_sample) / 1000)
            time.sleep((self.SAMPLETIME - self.tval_sample) / 1000)
        self.tval_before = time.time()*1000
      
  def distance(self,x, y, seg):
    # Project the point onto the line that defines the segment (dot product)
    proy = np.dot(seg.pf - seg.pi, np.array([x, y]) - seg.pi) / np.linalg.norm(seg.pf - seg.pi)
    
    # Position of the closest point with respect to pf
    L = np.linalg.norm(seg.pf - seg.pi)  # Length of the segment
    pc=0#define pc
    # There are three cases:
    if proy <= 0:
        # A) The projected distance is negative, meaning the closest point to the line
        # is outside the segment and the closest point on the segment is pi
        pc = seg.pi
    elif proy >= L:
        # B) In this case, the closest point to the line
        # is outside the segment and the closest point on the segment is pf
        pc = seg.pf
    else:
        # C) The closest point to the line is within the segment (default case). First, find the closest point
        pc = seg.pi + proy * (seg.pf - seg.pi) / np.linalg.norm(seg.pf - seg.pi)
    
    # Now compute the distance
    d = np.linalg.norm(np.array([x, y]) - pc)
    
    # Avoid singularity
    if d < 5:
        d = 5
    
    return d
  def stop(self):
    print("stop")
    
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
      self.PostionForControl[agent]=message
      #print(message)
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
  
  
  
 