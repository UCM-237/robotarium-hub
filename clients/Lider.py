# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging
import struct
import json
import csv
import math
import os

from pathlib import Path
from agent import Agent
from time import sleep, time

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = '4'
AGENT_IP = '192.168.10.68'
AGENT_CMD_PORT = 5561
AGENT_DATA_PORT = 5562
# Where the server is 
HUB_IP = '192.168.10.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556

LIMITES_ENTORNO = {
    "x_min": -0.8, "x_max": 0.90,
    "y_min": -0.9, "y_max": 0.004
}



class Robot:
  '''Encapsulates the communication with Arduino'''
  OP_MOVE_WHEEL: int = 2
  OP_STOP_WHEEL: int = 3
  OP_VEL_ROBOT: int = 5
  OP_CONF_PID: int = 9
  STRUCT_FORMAT = '<BBff'
  INIT_FLAG = 112
  VELOCIDAD_AVANCE = 1
  ANGULAR_VELOCITY_LEFT = 2 #Left angular velocity
  ANGULAR_VELOCITY_RIGHT = 0 #Righ angular velocity
  TOLERANCIA_DISTANCIA = 0.3
  TOLERANCIA_ANGULO = 0.40
  #ANTELACION_GIRO = 1.4 #Adelantamiento de la parada por el retardo entre la comunicación
  ANTELACION_GIRO = -2 #Adelantamiento de la parada por el retardo entre la comunicación
  ANTELACION_PARADA = 1

  running: bool = False
  connected: bool = False
  arduino: Serial = None
  thread: Thread = None
  listeners: list = None
  auto_discovery: list = ['Arduino', 'USB2.0-Serial']
  pose: list = [0, 0, 0]
  poseDesire: list = [0, 0, 0]
  state = 0
  inicio = 0
  fin = 0
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.start()

  def start(self) -> None:
    self.thread = Thread(target=self.controller).start()
    #self.thread = Thread(target=self.seguridad).start()
    #self.thread = Thread(target=self.collectData).start()
    return

  def controller(self) -> None:
    while (self.running == False):
      sleep(1)

    while True: 
      '''
      self.move_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(1)
      self.stop_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(0.5)
      self.move_wheels(self.ANGULAR_VELOCITY_LEFT, self.ANGULAR_VELOCITY_RIGHT)
      sleep(1.5)
      self.stop_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(0.5)
      self.move_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(0.85)
      self.stop_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(0.5)
      self.move_wheels(self.ANGULAR_VELOCITY_LEFT, self.ANGULAR_VELOCITY_RIGHT)
      sleep(0.75)
      self.stop_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      sleep(1)
      '''
      print("Rueda derecha")
      self.move_wheels(self.VELOCIDAD_AVANCE,0)
      sleep(2)
      print("Rueda izquierda")
      self.move_wheels(0,self.VELOCIDAD_AVANCE)
      sleep(2)

    
  def ajustarAngulo(self, yaw_deseado) -> bool:
    a = False
    error_angulo= yaw_deseado  - self.pose[2]
    error_angulo = self.normalizarYaw(error_angulo)
    while True:
      if abs(error_angulo) < self.TOLERANCIA_ANGULO:
        if a == True:
          print("Yaw ajustado")
        return a
      else :
        print("Encender motores")
        a = True
        self.agent.device.stop_wheels(self.VELOCIDAD_AVANCE ,self.VELOCIDAD_AVANCE)
        sleep(0.5) #Evita un error en el que el robot no conseguia empezar a guirar
        self.agent.device.move_wheels(self.ANGULAR_VELOCITY_LEFT, self.ANGULAR_VELOCITY_RIGHT)
        sleep(0.5) #Evita un error en el que se para en el angulo de antelación y se queda bloqueado
      error_angulo_antelcion = error_angulo + self.ANTELACION_GIRO 
      error_angulo_antelcion = self.normalizarYaw(error_angulo_antelcion)
      while abs(error_angulo_antelcion) > self.TOLERANCIA_ANGULO: #Hay que ajustar la orientación
        error_angulo_antelcion = yaw_deseado - self.pose[2] + self.ANTELACION_GIRO
        error_angulo_antelcion = self.normalizarYaw(error_angulo_antelcion)
      
      self.agent.device.stop_wheels(self.VELOCIDAD_AVANCE ,self.VELOCIDAD_AVANCE)
      init_pos=self.pose[2] #Ubicación retardada
      sleep(2) 
      end_pos=self.pose[2] #Ubicación real
      error_angulo= yaw_deseado  - end_pos
      error_angulo = self.normalizarYaw(error_angulo)
      print('Error')
      print(error_angulo) 
      print('error entre el yaw al mandar parar y el real')
      print(self.normalizarYaw(init_pos - end_pos))


  def ajustarDistancia(self, obj_x, obj_y) -> None:
    a = True 
    distancia_cm = math.sqrt((obj_x - self.pose[0])**2 + (obj_y - self.pose[1])**2)
    while distancia_cm > self.TOLERANCIA_DISTANCIA:
      if a == True: #Inicia los motores en la primera iteración y si en la anterior se ha ajustado la orientación
        print("avanzando")
        self.agent.device.move_wheels(self.VELOCIDAD_AVANCE, self.VELOCIDAD_AVANCE)
      yaw_deseado = math.atan2(obj_y - self.pose[1], obj_x - self.pose[0])
      yaw_deseado =  yaw_deseado - math.pi/2
      yaw_deseado = self.normalizarYaw(yaw_deseado)
      a = self.ajustarAngulo(yaw_deseado)
      distancia_cm = math.sqrt((obj_x - self.pose[0])**2 + (obj_y - self.pose[1])**2)
    self.agent.device.stop_wheels(self.VELOCIDAD_AVANCE ,self.VELOCIDAD_AVANCE)
    print("Destino")

  
  def normalizarYaw (self, yaw):
    # Normalizar la orientación (o el error) entre pi y -pi
    while yaw > math.pi: yaw -= 2 * math.pi
    while yaw < -math.pi: yaw += 2 * math.pi
    return yaw
  
  def unique_file_name(self, base_name, extension):
    index = 1
    while True:
        file_name = f"{base_name}{index}.{extension}"
        if not os.path.exists(file_name):
            return file_name
        index += 1

  def collectData(self) -> None:
    # Parametros del programa
    print("Collecting Data")
    data_array = []
    timeSnapInit = time()
    csv_file = self.unique_file_name('./Results/Lider', 'csv')
    csv_path = Path(csv_file)
    csv_path.parent.mkdir(parents=True, exist_ok = True)
    csv_header = ['x', 'y', 'yaw', 'state','timesnap'] 
    with open(csv_file, 'w', newline='') as file:
      writer = csv.writer(file)
      writer.writerow(csv_header)  # Escribe los encabezados en el archivo CSV
    while True:
      timeSnap = time() - timeSnapInit
      data_array =  [self.pose[0], self.pose[1], self.pose[2], self.state, timeSnap]
      with open(csv_file, 'a', newline='') as file:
          writer = csv.writer(file)
          writer.writerow(data_array)  # Escribe los datos en el archivo CSV
      sleep(0.1)


  def seguridad(self) -> None: 
    print("Seguridad")
    while (self.running == False):
      sleep(1)
    while True:
      #Calculo de la distancia de seguridad
      x_seguridad = self.pose[0] - self.TOLERANCIA_DISTANCIA 
      y_seguridad = self.pose[1] + self.TOLERANCIA_DISTANCIA 
      # Comprobar si la posición esta fuera de la de seguridad
      yawXmin =  - self.TOLERANCIA_ANGULO
      yawXmax =  self.TOLERANCIA_ANGULO
      if (self.pose[3] < yawXmin or self.pose[3]  > yawXmax): #El angulo del robot es hacia abajo 
        print("Angulo peligroso")
        if ((LIMITES_ENTORNO["x_min"] + self.TOLERANCIA_DISTANCIA) > x_seguridad): #Esta cerca del borde inferior
          print("Cerca de X")
          self.agent.device.stop_wheels(self.VELOCIDAD_AVANCE ,self.VELOCIDAD_AVANCE)
          return
      # if not (LIMITES_ENTORNO["y_min"] <= y_seguridad <= LIMITES_ENTORNO["y_max"]):
      #     print("Cerca de Y")
      #     self.agent.device.stop_wheels(0,0)
      #     return
        


  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    ports = list_ports.comports()
    for p in ports:
      try:
        #if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        self.arduino = Serial(p.device, baudrate=9600, timeout=5)
        #self.thread = Thread(target=self.update).start()
        self.connected = True
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot open Arduino.')
    else: 
      print('Arduino conected')
      self.running=True

  def move_wheels(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    message= struct.pack(self.STRUCT_FORMAT, 255, self.OP_MOVE_WHEEL, v_left, v_right)
    self.arduino.write(message)


  def stop_wheels(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    message= struct.pack(self.STRUCT_FORMAT, 255, self.OP_STOP_WHEEL, v_left, v_right)
    self.arduino.write(message)

  def on_data(self, topic: str, message: str) -> None:
    #print(f'{topic} {message}')
    if topic.startswith(f'data/{AGENT_ID}'):
      params = json.loads(message)
      x = float(params["x"])
      y = float(params["y"])
      yaw = float(params["yaw"])
      self.pose = [x, y , yaw]
    #print(f'Mi ubicacion es: x: {pose[0]}, y: {pose[1]}, yaw: {pose[2]}')
    #print(f'Mi deseada es: x: {poseDesire[0]}, y: {poseDesire[1]}, yaw: {poseDesire[2]}')

  def ArduinoSerialWrite(self,operation,len,data):
    head = (struct.pack('H',self.INIT_FLAG) + struct.pack('H',int(AGENT_ID)) + 
                struct.pack('H',operation) + struct.pack('H',len))
    message=head + data
    return message
  
  

if __name__ == "__main__":
  agent = Agent(
    device_class=Robot,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_ip = HUB_IP,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )


