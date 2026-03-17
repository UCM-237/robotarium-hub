# -*- coding: UTF-8 -*-
#!/bin/python3
"""
pi_agent_limits.py - Script principal del agente de límites.

Este módulo crea una instancia de `Robot` que comunica con
la placa Arduino, procesa operaciones de movimiento y se
integra con un agente ZMQ para recibir comandos y publicar
telemetría. El agente escucha topics como
`RobotariumData`, `control` y `data`.
"""
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
from threading import Event
import logging
import struct
import json
import random
import LimitsAlgorithm
import OrientationControl
from agent import Agent
import time
import xml.etree.ElementTree as ET

# Constante pi usada para conversiones de ángulos
PI = 3.14159


# Configure logs
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('robot_debug.log')
    ]
)

# # Who I am
# AGENT_ID = '2'
# AGENT_IP = '192.168.1.115'
# AGENT_CMD_PORT = 5561
# AGENT_DATA_PORT = 5562
# # Where the server is 
# HUB_IP = '192.168.1.109'
# HUB_CMD_PORT = 5555
# HUB_DATA_PORT = 5556


class Robot:
  '''Encapsulates the communication with Arduino'''
  OP_MOVE_ROBOT = 2
  OP_STOP_ROBOT = 3
  OP_TELEMETRY = 4
  OP_TURN_ROBOT = 5
  OP_SILENCE = 6
  OP_POSITION = 7
  OP_CONF_PID = 8
  OP_CONF_FF = 9
  OP_DONE = 10
  OP_MOVE_WHEELS = 11
  INIT_FLAG = 112
  connected: bool = False
  arduino: Serial = None
  Arduinothread: Thread = None
  ArenaRulesThread: Thread = None
  listeners: list = None
  auto_discovery: list = ['Arduino', 'USB2.0-Serial']
  communicationParameters = {}
  agentParameters = {}
  operations = {}
  AgentName = None
  AgentId = 5
 

  def __init__(self, agent=None) -> None:
    # `agent` es la instancia de `Agent` que envía/recibe mensajes mediante zmqtt.
    # Guardamos la referencia para que la clase Robot pueda publicar mediciones
    # y reaccionar a eventos de topics suscritos.
    self.agent=agent
    self.parsers = {
      self.OP_TELEMETRY: self.speed
    }
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
    # Inicializar LimitsAlgorithm (se configurará después de parseConfigurations())
    self.LimitsAlgorithm = None
    self.Position={}
    self.Pos=False
    self.ArenaLimitsReceived = False
    self.stopCommand = False
    self.operationFromRobotDone = Event()
    self.IgnoreControlCommunication = False
    # Parámetros para movimiento aleatorio
    self.maxRandomSpeed = 4  # Velocidad máxima para movimiento aleatorio (se configura desde XML)
    self.randomMovementInterval = 1  # Intervalo entre cambios de dirección aleatoria (segundos)
    self.L = 14.5  # Valor de ejemplo para L
    self.R = 3.35  # Valor de ejemplo para R
    self.A = [[self.L/(2*self.R), 1/self.R],
        [-self.L/(2*self.R), 1/self.R]]
    self.SAMPLETIME=200
    self.tval_before = 0
    self.tval_after= 0
    self.tval_sample = 0
    self.AgentName
    self.AgentId
    #self.orientationControl = OrientationControl.orientationControl(self.SAMPLETIME)



    self.agent = agent
  def parseConfigurations(self):
    '''Lee 'AgentConfiguration.xml' y llena los diccionarios de parámetros.

    Se espera que el archivo contenga secciones `<AgentParameters>` y
    `<CommunicationConfiguration>` además de los `RobotOperations` opcionales.
    Inicializa LimitsAlgorithm con la distancia de seguridad configurada.
    '''
    tree = ET.parse('AgentConfiguration.xml')
    root = tree.getroot()
    AgentParameters = root.find('AgentParameters')
    CommunicationParameters = root.find('CommunicationConfiguration')
    
    for agent in AgentParameters:
      self.agentParameters[agent.tag] = agent.text
      
    for communicationParameter in CommunicationParameters:
      self.communicationParameters[communicationParameter.tag] = communicationParameter.text
    for operation in root.find('RobotOperations'):
      self.operations[operation.tag] = operation.text
    
    # Obtener la distancia de seguridad, con valor por defecto de 0.15 m si no está configurada
    min_safety_dist = float(self.agentParameters.get('MinimumSafetyDistance', 0.15))
    self.LimitsAlgorithm = LimitsAlgorithm.LimitsAlgorithm(minimum_safety_distance=min_safety_dist)
    
    # Obtener la velocidad máxima para movimiento aleatorio (default: 0.1 m/s)
    self.maxRandomSpeed = float(self.agentParameters.get('MaxRandomSpeed', 4))
    
    self.AgentName = self.agentParameters['AgentName']
    self.AgentId= self.agentParameters['AgentId']
    logging.info(f"AgentID: {self.AgentId}")
    
    
  def connect(self) -> None:
    '''Open a new connection with an Arduino Board.'''
    logging.info(f"[connect] Intentando conectar con Arduino...")
    self.ArenaRulesThread = Thread(target=self.checkArenaRules).start()
        
    ports = list_ports.comports()
    for p in ports:
      try:
        if not p.description in self.auto_discovery: continue
        logging.info(f'Connecting to {p.description} in {p.device}')
        self.arduino = serial_for_url(p.device, baudrate=9600, timeout=5, write_timeout=5)
        self.Arduinothread = Thread(target=self.update).start()
        self.connected = True
        
        break
      except:
        logging.info(f'Cannot connect to {p.device}, trying another port')
    if self.arduino is None or not self.arduino.is_open:
      logging.error('Cannot open device.')


  def update(self) -> None:
    '''Parse data received from the robot'''
    logging.info('Starting Arduino update Arduinothread')
    
      
    while True:
      try:
        initFlag = int.from_bytes(self.arduino.read(size=4), byteorder='little')
        if initFlag == self.INIT_FLAG:

          id = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          operation = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          len = int.from_bytes(self.arduino.read(size=4), byteorder='little')
          data = self.arduino.read(size=len)
          if operation == self.OP_DONE:
            self.operationFromRobotDone.set()
          else:
            measurement = self.parse(operation, data)
            logging.debug(f'Message from {id}: op={operation}, {len} bytes received, data={measurement}')
            data={'telemetry','AGENT_ID'}
            topic = 'telemetry'
            self.agent.send_measurement(topic,measurement)
          
      except (ValueError, TypeError) as e:
        
        logging.debug('Ignoring invalid data from Arduino')
        print(e)
      
      except SerialException as e:
      
        logging.info('Disconnected from Arduino.')
        print(e)
  
  
  def checkArenaRules(self):
    
    #take the arenaLimits
    logging.info(f"[checkArenaRules] Aguardando límites de arena y posición inicial...")
    while(self.Pos==False and self.ArenaLimitsReceived == False):
      #self.agent.send("localization/RobotariumData","")
      logging.info("[checkArenaRules] Esperando posicion o limites...")
      time.sleep(1)
    logging.info(f"[checkArenaRules] Límites recibidos. Iniciando control de movimiento.")
    #First check the arena limits
    negativeAngle = False
    iteration = 0
    while(True):
      iteration += 1
      if len(self.Position)>0:
        #logging.info(f"Position:{self.Position[agent]}")
        robotData = self.Position[agent]
        #self.Position.pop(self.agentParameters['AgentId'])
        robotX = -float(robotData["x"])
        robotY = -float(robotData["y"])
        heading = -float(robotData["yaw"])
        logging.debug(f"[Iter {iteration}] Posición: X={robotX:.3f}, Y={robotY:.3f}, YAW={heading:.3f}")
        
        if heading < 0:
          heading = 2*PI + heading
        
        newHeading = self.LimitsAlgorithm.checkLimits(robotX,robotY,heading)
        logging.debug(f"Heading: {newHeading}")
        minDist = self.LimitsAlgorithm.minimumSafetyDistance
        
        angleError = round((newHeading - heading)*180/PI)
        if abs(angleError) > 180:
          angleError = 360 - abs(angleError) # turn shorter way
          
        if heading > 3*PI/2 and newHeading < PI/2:
          angleError = -angleError#turn right
        if heading < PI/2 and newHeading > 3*PI/2:
          angleError = -angleError
        
        if newHeading != 0:
          # DETECTÓ PELIGRO - Girar
          logging.warning(f"[Iter {iteration}]  IMITE DETECTADO - Distancia de seguridad: {minDist}m. Girando {angleError}° para evitar colisión.")
          self.IgnoreControlCommunication = True
          #stop command of moving the robot
          logging.info(f"[Iter {iteration}] Deteniendo robot...")
          self.move_robot(0,0)
          self.operationFromRobotDone.wait(timeout=0.1)
          if self.operationFromRobotDone.is_set():
            self.operationFromRobotDone.clear()
          #wait for op_done
          logging.info(f"[Iter {iteration}] Ejecutando giro de {angleError}°")
          self.turn_robot(angleError)
          self.operationFromRobotDone.wait()
          logging.info(f"[Iter {iteration}] Giro completado.")
        else:
          # Robot seguro - Movimiento aleatorio
          self.IgnoreControlCommunication = False
          logging.debug(f"[Iter {iteration}] ✓ Posición segura. Movimiento aleatorio activado.")
          self.random_movement()
        
        # Esperar un poco antes de la siguiente verificación
        time.sleep(self.randomMovementInterval)

          
          
          
  def random_movement(self) -> None:
    '''Mueve el robot lento en linea reacta y envía comando de movimiento al robot.'''
    # Generar velocidades aleatorias entre -maxRandomSpeed y +maxRandomSpeed
    v_left =11
    v_right = 12
    logging.debug(f"[random_movement] Enviando velocidades: IZQ={v_left:.4f} m/s, DER={v_right:.4f} m/s")
    self.move_robot(v_left,v_right)

  def move_robot(self, v_left, v_right) -> None:
    '''Set the wheels' speed setpoint'''
    len=8
    data=( struct.pack('f', v_left)+struct.pack('f',v_right))
    logging.debug(f"[move_robot] data: {data}")
    self.ArduinoSerialWrite(self.OP_MOVE_ROBOT,len,data)

  def move_wheels(self,pwm_left, pwm_right) -> None:
    '''Set the wheels' speed setpoint'''
    len=8
    data=(struct.pack('i', pwm_left) + struct.pack('i', pwm_right))
    self.ArduinoSerialWrite(self.OP_MOVE_WHEELS,len,data)
    
  def stop_robot(self,op) -> None:
    '''Stop the robot'''
    len=0
    data=(struct.pack('i', 0) + struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_STOP_ROBOT,len,data)
  def request_telemetry(self,op) -> None:
    '''Request telemetry data'''
    len=0
    data=(struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_TELEMETRY,len,data)
    
  def turn_robot(self, angle) -> None:
    '''Turn the robot a given angle'''
    #for easy implementation, the angle can be only 90, 180, 270, 360
    len=4
    data=(struct.pack('i', angle))
    self.ArduinoSerialWrite(self.OP_TURN_ROBOT,len,data)
    
    
  def silenceCommunication(self,op) -> None:
    '''Stop the communication with the robot'''
    len=0
    data=(struct.pack('i', 0))
    self.ArduinoSerialWrite(self.OP_SILENCE,len,data)
  def RequestPosition(self,op) -> None:
    '''Request the position of the robot'''
    len=0
    data=()
    self.ArduinoSerialWrite(self.OP_POSITION,len,data)
    
  def conf_PID(self,P_right, I_right, D_right, P_left, I_left, D_left) -> None:

    len=48#bytes
    data=(struct.pack('d', P_right) +
      struct.pack('d', I_right) +
      struct.pack('d', D_right) +
      struct.pack('d', P_left) +
      struct.pack('d', I_left) +
      struct.pack('d', D_left))
    
    self.ArduinoSerialWrite(self.OP_CONF_PID,len,data)
  def conf_FF(self,FF_right, FF_left) -> None:
    len=16
    data=(struct.pack('d', FF_right) + struct.pack('d', FF_left))
    self.ArduinoSerialWrite(self.OP_CONF_FF,len,data)
    
  def speed(self, data) -> dict:
    '''Parse speed from binary data'''
     # Definir el formato de deserialización
    fmt = 'ddii'  # dos valores double (d), seguidos de dos enteros (i)

    # Deserializar los datos usando struct.unpack
    v_left, v_right, pwm_left, pwm_right = struct.unpack(fmt, data)
    return {
      'w_left': v_left,
      'w_right': v_right,
      'pwm_left': pwm_left,
      'pwm_right': pwm_right
    }
  def batteryStatus(self, data) -> dict:
    pass
  
  def parse(self, operation: int, data: bytes) -> dict:
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    
    if self.stopCommand == True:
      return
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    self.operations[operation]['method'](**kwargs)


  def on_data(self, topic: str, message: str) -> None:
    #print(f'{topic} {message}')
    '''try
      _,agent,topic=topic.split('/')
    except ValueError:
      logging.error(f"Error al procesar el topic: {topic}. Se esperaban 3 campos")
      return '''
    try:
      full_json = json.loads(message)
      robot_id=str(self.agentParameters['AgentId'])
      if topic=="control"  and self.IgnoreControlCommunication == False:
        cmd = topic[len(f'control/{self.AgentName}')+1:].upper()
        # params = json.loads(message)
        self.exec(cmd, **data)
      elif topic=="Camara_0/position/5":
          payload=full_json.get("payload",{})
          pos=payload.get(robot_id)
          
          self.Position[agent]= pos
          self.Pos=True
          #print(f'position: {self.Position[agent]}')
      elif topic == "Camara_0/ArenaSize":
        self.ArenaLimitsReceived = True
        payload=full_json.get("payload",{})
        arena=payload.get("arenaSize")
        self.LimitsAlgorithm.addLimits(arena)
        #print(f"Arena Limits: {arena}")
      else:
        logging.error(f"Topic {topic} desconocido")
    
    except json.JSONDecodeError :
      logging.error(f"Error: el mensaje no es un json valido:{message}")
      self.move_robot(0,0)
    except Exception as e:
      logging.error(f"Error al procesar el topic {topic}:{e}")
      self.move_robot(0,0)	
    
       
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

  def ArduinoSerialWrite(self,operation,len,data):
    #in arduino initFlag is uint8_t, agent_id is uint8_t, operation is uint8_t, len is uint16_t
    
    #head=struct.pack('ii',self.INIT_FLAG,operation)+struct.pack('i',len)
    head = struct.pack('i', self.INIT_FLAG)+ struct.pack('i', int(self.agentParameters['AgentId']))+ struct.pack('i',operation) + struct.pack('i',len)
    message=head + data
    message += b'\n'
    bytes_written= self.arduino.write(message)
    logging.info(f"[ArduinoSerialWrite] written data:  {message}")		

if __name__ == "__main__":
  logging.info("=")
  logging.info(" INICIANDO ROBOT AGENT CON CONTROL DE LIMITES")
  # Instanciamos la clase Robot y cargamos configuración desde XM
  robot = Robot()
  logging.info("[main] Leyendo configuración desde AgentConfiguration.xml...") 
  robot.parseConfigurations()
  logging.info(f"[main] Configuración cargada: {robot.agentParameters['AgentName']}") 
  logging.info(f"[main] Velocidad aleatoria máxima: {robot.maxRandomSpeed}ms")
  logging.info(f"[main] Distancia de seguridad: {robot.LimitsAlgorithm.minimumSafetyDistance} m")
   # Creamos el agente ZMQ utilizando los parámetros obtenidos del XML. agente se suscribe automáticamente a los topics RobotariumData, contro# y data gracias a la implementación de la clase Agen\n
  logging.info("[main] Creando agente ZMQ...")
  agent = Agent(
    device_class=robot,
    id=robot.agentParameters['AgentName'],
    ip=robot.communicationParameters['AgentIp'],
    cmd_port=robot.communicationParameters['AgentCmdPort'],
    data_port=robot.communicationParameters['AgentDataPort'],
    hub_ip = robot.communicationParameters['HubIp'],
    hub_cmd_port = robot.communicationParameters['HubCmdPort'],
    hub_data_port = robot.communicationParameters['HubDataPort'],
  )
  robot.connect()
  #agent.device.on_data('control/RobotAgent1/telemetry', '{}')
  
  # agent = Agent(
  #   device_class=robot,
  #   id=AGENT_ID,
  #   ip=AGENT_IP,
  #   cmd_port=AGENT_CMD_PORT,
  #   data_port=AGENT_DATA_PORT,
  #   hub_ip = HUB_IP,
  #   hub_cmd_port=HUB_CMD_PORT,
  #   hub_data_port=HUB_DATA_PORT,
  # )
  
  
