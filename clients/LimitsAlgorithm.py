import numpy as np
import math

class segment:
    def __init__(self,pi,pf):
        self.pi=np.array(pi)
        self.pf=np.array(pf)
class LimitsAlgorithm:
    
    def __init__(self):
        self.ArenaLimits = {}
        self.newHeading = 0
        self.segmentOfTheRectangle = []
        self.minimumSafetyDistance = 10
        self.x=[]
        self.y=[]

    def addLimits(self,limits):
        self.ArenaLimits = limits
        self.x[1]=self.ArenaLimits["x1"]
        self.y[1]=self.ArenaLimits["y1"]
        self.x[2]=self.ArenaLimits["x2"]
        self.y[2]=self.ArenaLimits["y2"]
        self.x[3]=self.ArenaLimits["x3"]
        self.y[3]=self.ArenaLimits["y3"]
        self.x[4]=self.ArenaLimits["x4"]
        self.y[4]=self.ArenaLimits["y4"]
        self.segmentOfTheRectangle = self.computeSegments()
        
    def computeSegments(self):
        # coordinates of the rectangle
        # x1=self.ArenaLimits["x1"]
        # y1=self.ArenaLimits["y1"] 
        # x2=self.ArenaLimits["x2"]
        # y2=self.ArenaLimits["y2"]
        # x3=self.ArenaLimits["x3"]
        # y3=self.ArenaLimits["y3"]
        # x4=self.ArenaLimits["x4"]
        # y4=self.ArenaLimits["y4"]
        
        segmentOfTheRectangle =[                                  #S1       
            segment((self.x[1],self.y[1]),(self.x[2],self.y[2])),#segment 1              -------------------
            segment((self.x[2],self.y[2]),(self.x[3],self.y[3])),#segment 2              |                 |         
            segment((self.x[3],self.y[3]),(self.x[4],self.y[4])),#segment 3          s4  |                 | #s2
            segment((self.x[4],self.y[4]),(self.x[1],self.y[1]))  #segment 4             |                 |
                                                            #------------------- 
        ]             
        return segmentOfTheRectangle
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
    def checkLimits(self,robotX,robotY,heading):
        d=0
        newHeading=0
        dx = math.cos(heading)
        if dx < 0.08 and dx > -0.08:
            dx = 0
        dy = math.sin(heading)
        
        if dy < 0.08 and dy > -0.08:
            dy = 0
        if dx > 0:
            if dy > 0:
                print('The robot is moving to the upper righ')
                # compute the nearest point to the segment, always compare two segment becouse the noise of the robot
                if self.x[2] > robotX and self.y[2] > robotY:
                    print('The robot will move to the upper right corner of the rectangle.')
                    # compute the nearest point to the segment, always compare two segment becouse the noise of the robot
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading=-heading
                        else:
                            newHeading = -(math.pi - abs(heading))
                            
                else:
                    #stop the robot
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the left")          
            elif dy < 0:
                print('The robot is moving to the lower right corner.')
                if self.x[2] > robotX and self.y[1] < robotY:
                    print('The robot will move to the lower right corner of the rectangle.')
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading = -(math.pi - abs(heading))
                        else:
                            newHeading = -heading   
                else:
                    #stop the robot
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the right")
            else:
                print('The robot is moving to the right.')
                if self.x[2] > robotX:
                    print('The robot will move to the right wall of the rectangle.')
                    #in this case compare with 3 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    d3 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading = -heading
                        elif d == d2:
                            newHeading = -(math.pi - abs(heading))
                        else:
                            newHeading = -heading
        elif dx < 0:
            if dy > 0:
                print('The robot is moving to the upper left corner.')
                if self.x[1] < robotX and self.y[2] > robotY:
                    print('The robot will move to the upper left corner of the rectangle.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading = -heading
                        else:
                            newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left.TODO: STOP THE ROBOT and turn it to the right")
            elif dy < 0:
                print('The robot is moving to the lower left corner.')
                if self.x[1] < robotX and self.y[1] < robotY:
                    print('El robot se dirigirá hacia la esquina inferior izquierda del rectángulo.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading = -(math.pi - abs(heading))
                        else:
                            newHeading = -heading
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO: STOP THE ROBOT and turn it to the right")
            else:
                print('The robot is moving to the left.')
                #compare with the left wall of the rectangle and the three segments 
                if self.x[1] < robotX:
                    print('The robot will move to the left wall of the rectangle.')
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    d3 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            newHeading = -(math.pi - abs(heading))
                        elif d == d2:
                            newHeading = -heading
                        else:
                            newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO : STOP THE ROBOT and turn it to the right")  
        else:
            if dy > 0:
                print('The robot is moving up.')
                d = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                if d < self.minimumSafetyDistance:
                    newHeading = -heading
            elif dy < 0:
                print('The robot is moving down.')
                d = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                if d < self.minimumSafetyDistance:
                    newHeading = -(math.pi - abs(heading))
            else:
                print('El robot no se está moviendo, O no se detecta')
        return newHeading
       
                  
               