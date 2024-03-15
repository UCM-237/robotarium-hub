# -*- coding: UTF-8 -*-
#!/bin/python3
import array

from threading import Thread
import logging
import math
import time
import json
import numpy as np
from numpy.linalg import norm
from agent import Agent

from OrientationControl import orientationControl

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'LimitsAlgorithm'
AGENT_IP = '192.168.10.1'
AGENT_CMD_PORT = 5563
AGENT_DATA_PORT = 5564
AGENT_ID_NUM= '2'
# Where the server is 
HUB_IP = '192.168.10.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556
Position={}
def dot(v,w):
    x,y = v
    X,Y= w
    return x*X + y*Y

def length(v):
    x,y = v
    return math.sqrt(x*x + y*y )

def vector(b,e):
    x,y= b
    X,Y = e
    return (X-x, Y-y)

def unit(v):
    x,y = v
    mag = length(v)
    return (x/mag, y/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y= v
    return (x * sc, y * sc)

def add(v,w):
    x,y = v
    X,Y = w
    return (x+X, y+Y)


# Given a line with coordinates 'start' and 'end' and the
# coordinates of a point 'pnt' the proc returns the shortest 
# distance from pnt to the line and the coordinates of the 
# nearest point on the line.
#
# 1  Convert the line segment to a vector ('line_vec').
# 2  Create a vector connecting start to pnt ('pnt_vec').
# 3  Find the length of the line vector ('line_len').
# 4  Convert line_vec to a unit vector ('line_unitvec').
# 5  Scale pnt_vec by line_len ('pnt_vec_scaled').
# 6  Get the dot product of line_unitvec and pnt_vec_scaled ('t').
# 7  Ensure t is in the range 0 to 1.
# 8  Use t to get the nearest location on the line to the end
#    of vector pnt_vec_scaled ('nearest').
# 9  Calculate the distance from nearest to pnt_vec_scaled.
# 10 Translate nearest back to the start/end line. 
# Malcolm Kesson 16 Dec 2012

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist, nearest)
def point_to_line_dist(point, line):
    """Calculate the distance between a point and a line segment.

    To calculate the closest distance to a line segment, we first need to check
    if the point projects onto the line segment.  If it does, then we calculate
    the orthogonal distance from the point to the line.
    If the point does not project to the line segment, we calculate the 
    distance to both endpoints and take the shortest distance.

    :param point: Numpy array of form [x,y], describing the point.
    :type point: numpy.core.multiarray.ndarray
    :param line: list of endpoint arrays of form [P1, P2]
    :type line: list of numpy.core.multiarray.ndarray
    :return: The minimum distance to a point.
    :rtype: float
    """
    # unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) + 
        (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
    if is_betw_x and is_betw_y:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist
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
    self.SAMPLETIME=200
    self.OrientationControl = orientationControl(self.SAMPLETIME)
    self.orientationCorrected = False
    self.newHeadingRequired = False
    self.FirstComprobation = False
    self.newHeading=0
    
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
    minimumSafetyDistance=0.20
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
    robotX = -float(robotData["x"])
    robotY = -float(robotData["y"])
    heading = -float(robotData["yaw"])
    dx = math.cos(heading)
    if dx < 0.08 and dx > -0.08:
        dx = 0
    dy = math.sin(heading)
    if dy < 0.08 and dy > -0.08:
        dy = 0

    # coordinates of the rectangle
    x1=-self.ArenaLimits["x1"]
    y1=-self.ArenaLimits["y1"] 
    x2=-self.ArenaLimits["x2"]
    y2=-self.ArenaLimits["y2"]
    x3=-self.ArenaLimits["x3"]
    y3=-self.ArenaLimits["y3"]
    x4=-self.ArenaLimits["x4"]
    y4=-self.ArenaLimits["y4"]

    segmentOfTheRectangle =[                                  #S1       
        segment([x1,y1],[x2,y2]),#segment 1              -------------------
        segment([x2,y2],[x3,y3]),#segment 2              |                 |         
        segment([x3,y3],[x4,y4]),#segment 3          s4  |                 | #s2
        segment([x4,y4],[x1,y1])  #segment 4             |                 |
                                                        #------------------- 
    ]     
    # segmentOfTheRectangle =[                                  #S1       
    # segment((x2,y2),(x1,y1)),#segment 1              -------------------
    # segment((x3,y3),(x2,y2)),#segment 2              |                 |         
    # segment((x4,y4),(x3,y3)),#segment 3          s4  |                 | #s2
    # segment((x1,y1),(x4,y4))  #segment 4             |                 |
    #                                                     #------------------- 
    # ]                                                      #s3
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
                        print("robot collision minimumSafetyDistance")
                        if d == d1:
                            self.newHeading=-heading
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                            
                else:
                    #stop the robot
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the left")          
                    vel=0
                    velocity_robot=[w,vel]
                    self.angularWheelSpeed(angularWheel,velocity_robot)
                    self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
            elif dy < 0:
                print('The robot is moving to the lower right corner.')
                if x2 > robotX and y1 < robotY:
                    print('The robot will move to the lower right corner of the rectangle.')
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        print("robot collision minimumSafetyDistance")
                        
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
                        print("robot collision minimumSafetyDistance")
                        
                        if d == d1:
                            self.newHeading = -heading
                        elif d == d2:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading
        elif dx < 0:
            if dy > 0:
                print('The robot is moving to the upper left corner.')
                if x1 > robotX and y2 < robotY:
                    print('The robot will move to the upper left corner of the rectangle.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<minimumSafetyDistance:
                        print("robot collision minimumSafetyDistance")
                        
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
                        print("robot collision minimumSafetyDistance")
                        
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO: STOP THE ROBOT and turn it to the right")
                    vel=0
                    velocity_robot=[w,vel]
                    self.angularWheelSpeed(angularWheel,velocity_robot)
                    self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
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
                        print("robot collision minimumSafetyDistance")
                        
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        elif d == d2:
                            self.newHeading = -heading
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO : STOP THE ROBOT and turn it to the right")  
                    vel=0
                    velocity_robot=[w,vel]
                    self.angularWheelSpeed(angularWheel,velocity_robot)
                    self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
        else:
            if dy > 0:
                print('The robot is moving up.')
                d = self.distance(robotX, robotY, segmentOfTheRectangle[0])
                if d<minimumSafetyDistance:
                    print("robot collision minimumSafetyDistance")
                    
                    self.newHeading = -heading
            elif dy < 0:
                print('The robot is moving down.')
                d = self.distance(robotX, robotY, segmentOfTheRectangle[2])
                if d<minimumSafetyDistance:
                    print("robot collision minimumSafetyDistance")

                    self.newHeading = -(math.pi - abs(heading))
            else:
                print('El robot no se está moviendo, O no se detecta')
        self.FirstComprobation = True
        if self.newHeading != 0:
            #stop robot
            w=0
            vel=0
            velocity_robot=[w,vel]
            self.angularWheelSpeed(angularWheel,velocity_robot)
            
            self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
            time.sleep(5)
            self.newHeadingRequired=True
           
        
            
            
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
            w=0
        #now correct orientation 
        
            if self.FirstComprobation == True:
                if self.newHeadingRequired == True:
                    uniformVelocity=False
                    w =  self.OrientationControl.PID(heading, self.newHeading)
                    if w !=0.0:
                        velocity_robot=[w,vel]
                        self.angularWheelSpeed(angularWheel,velocity_robot)
                        self.agent.send('control/2/move',{'v_left':angularWheel[0],'v_right':angularWheel[1]})
                    else :
                        self.newHeadingRequired = False
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
    
    if x==seg.pi[0] and y == seg.pi[1]:
        return 0
    elif x == seg.pf[0] and y == seg.pi[1]:
        return 0
    P=np.array([x,y])
    line = [seg.pi, seg.pf]
    d=point_to_line_dist(P,line)
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
      
    import math


 
    
      


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
  
  
  
 