# -*- coding: UTF-8 -*-
#!/bin/python3
"""
Módulo del Agente Robot - Control de robot físico con protección de límites de arena

Este módulo implementa la clase Robot que:
1. Comunica con un microcontrolador Arduino por puerto serial
2. Monitorea la posición del robot en tiempo real
3. Aplica límites de arena para evitar colisiones
4. Interpreta comandos de control del sistema central (hub)
5. Envía datos de telemetría (velocidad, posición, etc.)
"""
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
from threading import Event
import logging
import struct
import json
import LimitsAlgorithm
import OrientationControl
from agent import Agent
import time
import xml.etree.ElementTree as ET

# Constante matemática pi
PI = 3.14159

# Configura logging a nivel INFO
logging.basicConfig(level=logging.INFO)


class Robot:
  '''Encapsula la comunicación con el microcontrolador Arduino del robot'''
  
  # Códigos de operación para comunicación con Arduino
  OP_MOVE_ROBOT = 2        # Mover el robot
  OP_STOP_ROBOT = 3        # Detener el robot
  OP_TELEMETRY = 4         # Solicitar datos de telemetría
  OP_TURN_ROBOT = 5        # Girar el robot
  OP_SILENCE = 6           # Silenciar comunicación
  OP_POSITION = 7          # Solicitar posición
  OP_CONF_PID = 8          # Configurar parámetros PID
  OP_CONF_FF = 9           # Configurar feedforward
  OP_DONE = 10             # Operación completada
  OP_MOVE_WHEELS = 11      # Mover ruedas directamente
  INIT_FLAG = 112          # Flag de sincronización con Arduino
  
  connected: bool = False                    # ¿Conectado a Arduino?
  arduino: Serial = None                     # Conexión serial con Arduino
  Arduinothread: Thread = None               # Thread para leer datos del Arduino
  ArenaRulesThread: Thread = None            # Thread para verificar límites
  listeners: list = None                     # Listeners de eventos
  auto_discovery: list = ['Arduino', 'USB2.0-Serial']  # Puertos a buscar
  communicationParameters = {}               # Parámetros de comunicación desde XML
  agentParameters = {}                       # Parámetros del agente desde XML
  operations = {}                            # Mapeo de operaciones disponibles
 

  def __init__(self, agent: Agent) -> None:
    """
    Constructor del Robot
    
    Args:
      agent: Referencia al agente para comunicación con el hub
    """
    # Mapea códigos de operación a funciones parsers
    self.parsers = {
      self.OP_TELEMETRY: self.speed
    }
    
    # Mapea nombres de operaciones a sus implementaciones
    self.operations = {
      'MOVE': { 'id': self.OP_MOVE_ROBOT, 'method': self.move_robot },
      'STOP': { 'id': self.OP_STOP_ROBOT, 'method': self.stop_robot },
      'TELEMETRY': { 'id': self.OP_TELEMETRY, 'method': self.request_telemetry },
      'TURN': { 'id': self.OP_TURN_ROBOT, 'method': self.turn_robot },
      'SILENCE': { 'id': self.OP_SILENCE, 'method': self.silenceCommunication },
      'POSITION': { 'id': self.OP_POSITION, 'method': self.RequestPosition },
      'PID' : { 'id': self.OP_CONF_PID,  'method' : self.conf_PID },
      'FF' : { 'id': self.OP_CONF_FF,  'method' : self.conf_FF },
      'MOVE_WHEELS' : { 'id': self.OP_MOVE_WHEELS,  'method' : self.move_wheels }
    }
    
    # Instancia el algoritmo de límites
    self.LimitsAlgorithm = LimitsAlgorithm.LimitsAlgorithm()
    
    # Almacena la posición actual del robot por agente_id
    self.Position = {}
    
    # Control de estados
    self.ArenaLimitsReceived = False                     # ¿Se recibieron los límites de arena?
    self.stopCommand = False                             # Flag de parada
    self.operationFromRobotDone = Event()                # Evento cuando Arduino completa operación
    self.IgnoreControlCommunication = False              # Flag para ignorar comandos durante giros
    
    # Parámetros del robot (distancia entre ruedas y radio)
    self.L = 14.5  # Distancia entre ruedas (cm)
    self.R = 3.35  # Radio de las ruedas (cm)
    # Matriz A: convierte [velocidad_lineal, velocidad_angular] a [w_left, w_right]
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    
    # Parámetros de tiempo
    self.SAMPLETIME = 200  # Período de muestreo en ms
    self.tval_before = 0
    self.tval_after = 0
    self.tval_sample = 0
    self.AgentName = ""
    
    # Controlador de orientación (descomentado si se necesita)
    # self.orientationControl = OrientationControl.orientationControl(self.SAMPLETIME)
    
    # Referencia al agente para comunicación con el hub
    self.agent = agent
  def parseConfigurations(self):
    """Carga parámetros desde el archivo XML de configuración"""
    tree = ET.parse('AgentConfiguration.xml')
    root = tree.getroot()
    
    # Lee los parámetros del agente
    AgentParameters = root.find('AgentParameters')
    # Lee los parámetros de comunicación
    CommunicationParameters = root.find('CommunicationConfiguration')
    
    # Almacena parámetros del agente en un diccionario
    for agent in AgentParameters:
      self.agentParameters[agent.tag] = agent.text
      
    # Almacena parámetros de comunicación en un diccionario
    for communicationParameter in CommunicationParameters:
      self.communicationParameters[communicationParameter.tag] = communicationParameter.text
    
    # Lee operaciones adicionales del robot (si existen)
    for operation in root.find('RobotOperations'):
      self.operations[operation.tag] = operation.text
    
    # Guarda el nombre del agente
    self.AgentName = self.agentParameters['AgentName']
    
    
  def connect(self) -> None:
    '''Abre una nueva conexión con un Arduino'''
    # Inicia thread para verificar límites de arena
    self.ArenaRulesThread = Thread(target=self.checkArenaRules).start()
    
    # Busca y conecta con Arduino
    ports = list_ports.comports()
    for p in ports:
      try:
        # Busca puertos Arduino conocidos
        if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        
        # Abre conexión serial
        self.arduino = serial_for_url(p.device, baudrate=9600, timeout=5, write_timeout=5)
        
        # Inicia thread para leer datos del Arduino
        self.Arduinothread = Thread(target=self.update).start()
        self.connected = True
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    
    # Si no se pudo conectar, registra error
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot open device.')


  def update(self) -> None:
    '''Thread que procesa datos recibidos del robot (Arduino)'''
    logging.info('Starting Arduino update thread')
    
    # Bucle infinito para recibir y procesar mensajes del Arduino
    while True:
      try:
        # Lee el flag de sincronización (primeros 4 bytes)
        initFlag = int.from_bytes(self.arduino.read(size=4), byteorder='little')
        if initFlag == self.INIT_FLAG:
          # Lee la información del mensaje
          id = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          operation = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          len = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          data = self.arduino.read(size=len)
          
          if operation == self.OP_DONE:
            # Arduino completó una operación
            self.operationFromRobotDone.set()
          else:
            # Procesa los datos recibidos según el tipo de operación
            measurement = self.parse(operation, data)
            logging.debug(f'Message from {id}: op={operation}, {len} bytes received, data={measurement}')
            
            # Envía los datos al hub
            topic = 'telemetry'
            self.agent.send(topic, measurement)
          
      except (ValueError, TypeError) as e:
        # Ignora datos inválidos
        logging.debug('Ignoring invalid data from Arduino')
        print(e)
      
      except SerialException as e:
        # Se perdió la conexión con Arduino
        logging.info('Disconnected from Arduino.')
        print(e)
  
  
  def checkArenaRules(self):
    '''Thread que continuamente verifica que el robot no colisione con los límites'''
    
    """
    Hilo principal de seguridad. Monitoriza la posición del robot en tiempo real
    y decide si debe intervenir para evitar que el robot salga de la arena.
    
    Si el robot se acerca a un límite (definido en LimitsAlgorithm), activa 
    el modo de recuperación, detiene el movimiento externo y gira el robot.
    """

    # Espera a recibir la posición del robot y los límites de arena
    while(self.agentParameters['AgentId'] not in self.Position or self.ArenaLimitsReceived == False):
      self.agent.send("localization/RobotariumData", "")
      time.sleep(1)
    
    # Bucle principal de verificación
    while(True):
      if len(self.Position) > 0:
        # Obtiene los datos de posición del robot
        robotData = json.loads(self.Position[self.agentParameters['AgentId']])
        self.Position.pop(self.agentParameters['AgentId'])
        
        # Extrae coordenadas y rumbo (negados según el sistema de coordenadas)
        robotX = -float(robotData["x"])
        robotY = -float(robotData["y"])
        heading = -float(robotData["yaw"])
        
        # Normaliza el ángulo al rango [0, 2π]
        if heading < 0:
          heading = 2*PI + heading
        
        # Verifica los límites de arena y obtiene nuevo rumbo si es necesario
        newHeading = self.LimitsAlgorithm.checkLimits(robotX, robotY, heading)
        
        # Calcula el error angular en grados
        angleError = round((newHeading - heading) * 180 / PI)
        
        # Si es mayor que 180°, gira en sentido inverso (camino más corto)
        if abs(angleError) > 180:
          angleError = 360 - abs(angleError)
          
        # Detecta transiciones alrededor de 0°/360°
        if heading > 3*PI/2 and newHeading < PI/2:
          angleError = -angleError  # Gira hacia la derecha
        if heading < PI/2 and newHeading > 3*PI/2:
          angleError = -angleError
        
        # Si se necesita giro de corrección
        if newHeading != 0:
          # Ignora comandos de control ext erno durante la corrección
          self.IgnoreControlCommunication = True
          
          # Detiene el movimiento del robot
          self.move_robot(0, 0)
          self.operationFromRobotDone.wait(timeout=0.1)
          if self.operationFromRobotDone.is_set():
            self.operationFromRobotDone.clear()
          
          # Gira el robot para corregir la trayectoria
          self.turn_robot(angleError)
          self.operationFromRobotDone.wait()
        else:
          # El robot está seguro, permite comandos de control externo
          self.IgnoreControlCommunication = False

      

          
          
          
  def move_robot(self, v_left, v_right) -> None:
    """
    Convierte velocidades de cuerpo (m/s y rad/s) a comandos para las ruedas
    y los envía al Arduino por puerto serie.
    
    Args:
        linear_vel (float): Velocidad hacia adelante deseada.
        angular_vel (float): Velocidad de giro deseada.
    """
    len = 16  # 2 doubles = 16 bytes
    data = struct.pack('<dd', v_left, v_right)
    self.ArduinoSerialWrite(self.OP_MOVE_ROBOT, len, data)

  def move_wheels(self, pwm_left, pwm_right) -> None:
    '''Establece el PWM directo de las ruedas'''
    len = 8  # 2 integers = 8 bytes
    data = (struct.pack('i', pwm_left) + struct.pack('i', pwm_right))
    self.ArduinoSerialWrite(self.OP_MOVE_WHEELS, len, data)
    
  def stop_robot(self, op) -> None:
    '''Detiene el robot'''
    len = 0
    data = (struct.pack('i', 0) + struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_STOP_ROBOT, len, data)
    
  def request_telemetry(self, op) -> None:
    '''Solicita datos de telemetría (velocidades, posición)'''
    len = 0
    data = (struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_TELEMETRY, len, data)
    
  def turn_robot(self, angle) -> None:
    '''Gira el robot un ángulo especificado en grados'''
    len = 4
    data = (struct.pack('i', angle))
    self.ArduinoSerialWrite(self.OP_TURN_ROBOT, len, data)
    
    
  def silenceCommunication(self, op) -> None:
    '''Detiene la comunicación con el robot'''
    len = 0
    data = (struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_SILENCE, len, data)
    
  def RequestPosition(self, op) -> None:
    '''Solicita la posición actual del robot'''
    len = 0
    data = ()
    self.ArduinoSerialWrite(self.OP_POSITION, len, data)
    
  def conf_PID(self, P_right, I_right, D_right, P_left, I_left, D_left) -> None:
    '''Configura los parámetros PID para ambas ruedas'''
    len = 48  # 6 doubles = 48 bytes
    data = (struct.pack('d', P_right) +
      struct.pack('d', I_right) +
      struct.pack('d', D_right) +
      struct.pack('d', P_left) +
      struct.pack('d', I_left) +
      struct.pack('d', D_left))
    self.ArduinoSerialWrite(self.OP_CONF_PID, len, data)
    
  def conf_FF(self, FF_right, FF_left) -> None:
    '''Configura los parámetros de feedforward'''
    len = 16  # 2 doubles = 16 bytes
    data = (struct.pack('d', FF_right) + struct.pack('d', FF_left))
    self.ArduinoSerialWrite(self.OP_CONF_FF, len, data)
    
  def speed(self, data) -> dict:
    '''Interpreta datos binarios de velocidad del Arduino'''
    # Formato: 2 doubles (velocidades) + 2 integers (PWM)
    fmt = 'ddii'
    v_left, v_right, pwm_left, pwm_right = struct.unpack(fmt, data)
    return {
      'w_left': v_left,
      'w_right': v_right,
      'pwm_left': pwm_left,
      'pwm_right': pwm_right
    }
    
  def batteryStatus(self, data) -> dict:
    '''Interpreta estado de batería (sin implementar)'''
    pass
  
  def parse(self, operation: int, data: bytes) -> dict:
    '''Decodifica datos según el tipo de operación'''
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    '''Ejecuta una operación del robot'''
    # No ejecuta si hay un comando de parada activo
    if self.stopCommand == True:
      return
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    # Ejecuta el método asociado a la operación
    self.operations[operation]['method'](**kwargs)


  def on_data(self, topic: str, message: str) -> None:
    """
    Manejador de mensajes entrantes vía ZMQ (desde el servidor/hub).
    
    Args:
        topic (str): Canal del mensaje ('data' para posición, 'control' para órdenes).
        message (str): JSON con los datos (coordenadas x, y, theta o comandos).
    """
    # Procesa comandos de control dirigidos a este agente
    if topic.startswith('control/') and self.IgnoreControlCommunication == False:
      # Extrae el nombre del comando del topic
      cmd = topic[len(f'control/{self.AgentName}') + 1:].upper()
      # Parsea los parámetros como JSON
      params = json.loads(message)
      # Ejecuta el comando
      self.exec(cmd, **params)
    else:
      # Procesa otros datos (posición, límites de arena, etc.)
      _, agent, topic = topic.split('/')
      if topic == "position":
        # Almacena la posición del robot
        if int(agent) == int(self.agentParameters["AgentId"]):
          self.Position[agent] = message
      elif topic == "ArenaSize":
        # Recibe los límites de la arena
        self.ArenaLimitsReceived = True
        self.LimitsAlgorithm.addLimits(json.loads(message))
               
       
  def angularWheelSpeed(self, w_wheel, velocity_robot):
    '''Convierte [velocidad_lineal, velocidad_angular] a velocidades de ruedas'''
    fila = 2
    columna = 2
    aux = 0

    # Inicializa velocidades de ruedas a cero
    for i in range(2):
        w_wheel[i] = 0
    
    # Aplica matriz de transformación: w = A * v
    for i in range(fila):
        for j in range(columna):
            aux += (self.A[i][j] * velocity_robot[j])
        
        w_wheel[i] = aux

        # Ignora velocidades pequeñas (ruido)
        if w_wheel[i] < 0 and w_wheel[i] > -6:
            w_wheel[i] = 0.0
        elif w_wheel[i] > 0 and w_wheel[i] < 6:
            w_wheel[i] = 0.0
        
        aux = 0    

  def ArduinoSerialWrite(self, operation, len, data):
    """
    Empaqueta y envía una trama binaria estructurada al Arduino.
    
    Args:
        operation (int): Código de operación (ej. OP_MOVE_ROBOT).
        length (int): Tamaño de los datos.
        data (bytes): Payload con los parámetros de la operación.
    """
    # El protocolo es:
    # - initFlag: uint32 (4 bytes) = 112
    # - agent_id: uint32 (4 bytes)
    # - operation: uint32 (4 bytes)
    # - len: uint32 (4 bytes) = tamaño de data
    # - data: variable
    # - '\n': terminador
    
    # Construye el encabezado del mensaje
    head = struct.pack('iii', self.INIT_FLAG, int(self.agentParameters['AgentId']), operation) + struct.pack('i', len)
    
    # Concatena encabezado, datos y terminador
    message = head + data
    message += b'\n'
    
    # Escribe el mensaje en el puerto serial
    bytes_written = self.arduino.write(message)
		

if __name__ == "__main__":
  # Crea instancia del robot
  robot = Robot
  
  # Carga la configuración desde el archivo XML
  robot.parseConfigurations(robot)
  
  # Crea el agente con los parámetros leídos del archivo XML
  agent = Agent(
    device_class=robot,
    id=robot.agentParameters['AgentName'],
    ip=robot.communicationParameters['AgentIp'],
    cmd_port=int(robot.communicationParameters['AgentCmdPort']),
    data_port=int(robot.communicationParameters['AgentDataPort']),
    hub_ip=robot.communicationParameters['HubIp'],
    hub_cmd_port=int(robot.communicationParameters['HubCmdPort']),
    hub_data_port=int(robot.communicationParameters['HubDataPort']),
  )
  
  # El agente se mantiene ejecutándose en background con sus threads