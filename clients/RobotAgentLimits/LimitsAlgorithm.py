import numpy as np
import math

class segment:
    """Representa un segmento de línea entre dos puntos"""
    def __init__(self, pi, pf):
        """
        Args:
            pi: Punto inicial [x, y]
            pf: Punto final [x, y]
        """
        self.pi = np.array(pi)
        self.pf = np.array(pf)

class LimitsAlgorithm:
    """
    Algoritmo que verifica si un robot está dentro de los límites de una arena
    rectangular. Si el robot se acerca demasiado a una pared, determina un 
    nuevo rumbo de escape para evitar colisiones.
    """
    
    def __init__(self):
        """Inicializa el algoritmo con parámetros por defecto"""
        self.ArenaLimits = {}  # Límites de la arena
        self.newHeading = 0  # Nuevo rumbo si se detecta peligro de colisión
        self.segmentOfTheRectangle = []  # Los 4 segmentos de las paredes
        self.minimumSafetyDistance = 0.3  # Distancia mínima segura a las paredes (metros)
        self.x = [None]*5  # Coordenadas X de los 4 vértices (índices 1-4)
        self.y = [None]*5  # Coordenadas Y de los 4 vértices (índices 1-4)

    def addLimits(self, limits):
        """
        Define los límites de la arena rectangular
        
        Args:
            limits: Diccionario con x1,y1,x2,y2,x3,y3,x4,y4 (vértices de la arena)
        """
        self.ArenaLimits = limits
        # Almacena las coordenadas de los 4 vértices
        self.x[1] = -self.ArenaLimits["x1"]
        self.y[1] = -self.ArenaLimits["y1"]
        self.x[2] = -self.ArenaLimits["x2"]
        self.y[2] = -self.ArenaLimits["y2"]
        self.x[3] = -self.ArenaLimits["x3"]
        self.y[3] = -self.ArenaLimits["y3"]
        self.x[4] = -self.ArenaLimits["x4"]
        self.y[4] = -self.ArenaLimits["y4"]
        # Crea los 4 segmentos que forman el rectángulo
        self.segmentOfTheRectangle = self.computeSegments()
        
    def computeSegments(self):
        """
        Crea los 4 segmentos (paredes) del rectángulo
        Devuelve:
            Lista con los 4 segmentos que forman el rectángulo
        """
        segmentOfTheRectangle = [
            # Pared superior (S1)
            segment([self.x[1], self.y[1]], [self.x[2], self.y[2]]),
            # Pared derecha (S2)
            segment((self.x[2], self.y[2]), [self.x[3], self.y[3]]),
            # Pared inferior (S3) 
            segment([self.x[3], self.y[3]], [self.x[4], self.y[4]]),
            # Pared izquierda (S4)
            segment([self.x[4], self.y[4]], [self.x[1], self.y[1]])
        ]
        return segmentOfTheRectangle
    
    def distance(self, x, y, seg):
        """
        Calcula la distancia desde un punto a un segmento de línea
        
        Args:
            x, y: Coordenadas del punto
            seg: Segmento para calcular distancia
            
        Devuelve:
            Distancia del punto al segmento
        """
        if x == seg.pi[0] and y == seg.pi[1]:
            return 0
        elif x == seg.pf[0] and y == seg.pi[1]:
            return 0
        P = np.array([x, y])
        line = [seg.pi, seg.pf]
        d = self.point_to_line_dist(P, line)
        return d
    def point_to_line_dist(self,point, line):
        """Calculate the distance between a point and a line segment.

        To calculate the closest distance to a line segment, we first need to check
        if the point projects onto the line segment.  If it does, then we calculate
        the orthogonal distance from the point to the line.
        If the point does not project to the line segment, we calculate the 
        distance to both endpoints and take the shortest distance.

        Args:
            point: Array numpy [x, y] describiendo el punto
            line: Lista de arrays [P1, P2] con los extremos del segmento
            
        Devuelve:
            float: Distancia mínima al segmento
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
    def checkLimits(self,robotX,robotY,heading):
        """
    Calcula si el robot está en zona de colisión y determina el nuevo ángulo de escape.
    
    Args:
        robotX, robotY (float): Posición actual del robot.
        heading (float): Orientación actual en radianes.
    
    Returns:
        float: El ángulo 'newHeading' al que debe apuntar el robot para alejarse del límite.
    """
        d=0
        self.newHeading = 0
        dx = math.cos(heading)
        if dx < 0.08 and dx > -0.08:
            dx = 0
            
        dy = math.sin(heading)
        if dy < 0.08 and dy > -0.08:
            dy = 0
        if dx > 0:
            if dy > 0:
                #print('The robot is moving to the upper righ')
                # compute the nearest point to the segment, always compare two segment becouse the noise of the robot
                if self.x[2] < robotX and self.y[2] > robotY:
                    # Calcula distancia a las 2 paredes cercanas
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    #take the minimum distance
                    d = min(d1, d2)
                    # Si está muy cerca, calcula nuevo rumbo
                    if d < self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = 2*math.pi - heading
                        else:
                            self.newHeading = 2*math.pi - heading 
                            
                else:
                    #stop the robot
                    #print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the left")          
                    pass
            elif dy < 0:
                #print('The robot is moving to the lower right corner.')
                if self.x[3] < robotX and self.y[3] < robotY:
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    d = min(d1, d2)
                    if d < self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = 2*math.pi - heading 
                        else:
                            self.newHeading = 5*math.pi - 2*heading 
                else:
                    # Posible colisión con pared derecha
                    print("probably collision with the wall at right. TODO: STOP THE ROBOT and turn it to the right")
                    
            else:  # Movimiento solo hacia la derecha
                if self.x[2] < robotX:
                    #print('The robot will move to the right wall of the rectangle.')
                    #in this case compare with 3 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[1])
                    d3 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -heading
                        elif d == d2:
                            self.newHeading = -(math.pi - abs(heading))
                        else:
                            self.newHeading = -heading
                            
        elif dx < 0:  # Movimiento hacia la izquierda
            if dy > 0:  # Esquina superior izquierda
                if self.x[1] > robotX and self.y[1] > robotY:
                    #print('The robot will move to the upper left corner of the rectangle.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = 2*math.pi - heading
                        else:
                            self.newHeading = math.pi - heading
                else:
                    #stop the robot
                    print("probably collision with the wall at left.TODO: STOP THE ROBOT and turn it to the right")
                    
            elif dy < 0:  # Esquina inferior izquierda
                if self.x[4] > robotX and self.y[4] < robotY:
                    #print('El robot se dirigirá hacia la esquina inferior izquierda del rectángulo.')
                    #in this case compare with 2 segments
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    #take the minimum distance
                    d = min(d1, d2)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = 3*math.pi - heading
                        else:
                           
                            self.newHeading = 2*math.pi - heading
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO: STOP THE ROBOT and turn it to the right")
                    
            else:  # Movimiento solo hacia la izquierda
                if self.x[1] > robotX:
                    print('The robot will move to the left wall of the rectangle.')
                    d1 = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                    d2 = self.distance(robotX, robotY, self.segmentOfTheRectangle[3])
                    d3 = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                    #take the minimum distance
                    d = min(d1, d2, d3)
                    if d<self.minimumSafetyDistance:
                        if d == d1:
                            self.newHeading = -(math.pi - abs(heading))
                        elif d == d2:
                            self.newHeading = 0
                        else:
                            self.newHeading = -(math.pi - abs(heading))
                else:
                    #stop the robot
                    print("probably collision with the wall at left. TODO : STOP THE ROBOT and turn it to the right")  
                    
        else:  # Sin movimiento horizontal (solo vertical)
            if dy > 0:
                # El robot se mueve hacia arriba
                d = self.distance(robotX, robotY, self.segmentOfTheRectangle[0])
                if d < self.minimumSafetyDistance:
                    self.newHeading = -heading
                    
            elif dy < 0:
                # El robot se mueve hacia abajo
                d = self.distance(robotX, robotY, self.segmentOfTheRectangle[2])
                if d < self.minimumSafetyDistance:
                    self.newHeading = -(math.pi - abs(heading))
            else:
                # El robot no se está moviendo
                print('The robot is not moving. or it is not detected')
                
        return self.newHeading
       
                  
               