# -*- coding: UTF-8 -*-
#!/bin/python3
"""
Agent Module - Comunicación entre agentes y hub usando ZMQ
Este módulo implementa un sistema de comunicación distribuida donde:
- Los agentes se registran con un hub central
- Envían y reciben mensajes a través de sockets ZMQ
- Una clase Device puede conectarse para manejar datos entrantes
"""
import logging
import time
import zmq
from typing import Protocol
from threading import Thread

# Configura el nivel de logging a DEBUG para ver todos los mensajes
logging.basicConfig(level=logging.DEBUG)

class Agent:
  '''Pre-declare class Agent'''

class Device(Protocol):
  """Protocolo que define la interfaz que debe implementar cualquier dispositivo"""

  def __init__(self, agent: Agent) -> None:
    '''El constructor recibe un agente como referencia'''

  def connect(self) -> None:
    '''Establish a connection with the hardware'''

  def on_data(self, topic: str, message: str) -> None:
    '''Handle incoming data'''


class Agent:

  def __init__(self, device_class: Device, id: str, ip: str, cmd_port: int=5555, data_port:int=5556,
               hub_ip: str='127.0.0.1', hub_cmd_port: int=5555, hub_data_port: int=5556) -> None:
    """
    Constructor del Agente
    
    Args:
      device_class: Clase que implementa la interfaz Device (ej: Robot)
      id: Identificador único del agente
      ip: Dirección IP donde escucha este agente
      cmd_port: Puerto para comandos (por defecto 5555)
      data_port: Puerto para enviar datos (por defecto 5556)
      hub_ip: IP del hub central (por defecto localhost)
      hub_cmd_port: Puerto de comandos del hub
      hub_data_port: Puerto de datos del hub
    """
    """
    Constructor del Agente
    
    Args:
      device_class: Clase que implementa la interfaz Device (ej: Robot)
      id: Identificador único del agente
      ip: Dirección IP donde escucha este agente
      cmd_port: Puerto para comandos (por defecto 5555)
      data_port: Puerto para enviar datos (por defecto 5556)
      hub_ip: IP del hub central (por defecto localhost)
      hub_cmd_port: Puerto de comandos del hub
      hub_data_port: Puerto de datos del hub
    """
    self.id = id
    self.ip = ip
    self.cmd_port = cmd_port
    self.data_port = data_port
    self.hub_ip = hub_ip
    self.hub_cmd_port = hub_cmd_port
    self.hub_data_port = hub_data_port
    
    # Crea el contexto ZMQ
    context = zmq.Context()
    
    # Socket REQ para enviar comandos al hub
    self.control = context.socket(zmq.REQ)
    
    # Socket PUB para publicar datos que otros agentes pueden recibir
    self.data = context.socket(zmq.PUB)
    self.data.bind(f'tcp://*:{data_port}')
    
    # Socket SUB para suscribirse a datos del hub
    self.hub_data = context.socket(zmq.SUB)
    
    # Instancia el dispositivo (Robot/Arduino)
    self.device = device_class(agent=self)
    self.device.connect()
    
    # Registra este agente con el hub
    self.register()
    

  def _get_data_url(self):
    """Construye la URL para publicar datos"""
    return f'tcp://{self.ip}:{self.data_port}'

  def _get_cmd_url(self):
    """Construye la URL para escuchar comandos"""
    return f'tcp://{self.ip}:{self.cmd_port}'

  def _get_hub_data_url(self):
    """Construye la URL para recibir datos del hub"""
    return f'tcp://{self.hub_ip}:{self.hub_data_port}'

  def _get_hub_cmd_url(self):
    """Construye la URL para enviar comandos al hub"""
    return f'tcp://{self.hub_ip}:{self.hub_cmd_port}'


  def register(self) -> None:
    '''Register to the hub'''
    print(f'Registering agent')
    self.control.connect(self._get_hub_cmd_url())
    
    # Envía mensaje "hello" con su URL de datos
    self.control.send_json({
      'operation': 'hello',
      'source_id': self.id,
      'payload': {
        'url' : self._get_data_url()
      },
      'timestamp': 1000*time.time(),
    })
    
    # Espera respuesta del hub
    response = self.control.recv_json()
    self.connected = response['result'] == 'ok'
    
    # Inicia un thread para escuchar datos del hub
    Thread(target=self.listen).start()


  def listen(self) -> None:
    '''Receive data from other agents'''
    logging.debug(f'Connecting to hub at {self._get_hub_data_url()}')
    self.hub_data.connect(self._get_hub_data_url())

    # Se suscribe a mensajes con tema "data"
    logging.debug('Subscribing to data')
    self.hub_data.setsockopt(zmq.SUBSCRIBE, b'data')

    # Se suscribe a comandos de control dirigidos a este agente
    logging.debug('Subscribing to control')
    self.hub_data.setsockopt(zmq.SUBSCRIBE, b'control/RobotAgent1')

    # Bucle infinito para recibir mensajes
    while True:
      topic = self.hub_data.recv_string()
      message = self.hub_data.recv_string()
      # Pasa los datos al dispositivo para procesarlos
      self.device.on_data(topic, message)

  def send(self, topic: str, data: dict) -> None:
    '''Send data to a topic'''
    self.data.send_string(topic, flags=zmq.SNDMORE)
    self.data.send_json(data)


  def send_measurement(self, topic,data) -> None:
    '''Send a new measurement'''
    payload ={self.id:data}
    self.data.send_string('data', flags=zmq.SNDMORE)
    self.data.send_json({
      'topic': 'telemetry',
       'payload':payload,
      'timestamp': 1000*time.time(),
    })


if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = Agent()
    agent.register()
    logging.info(f'Agent {agent.id} is listening')