# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import time
import zmq

import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import json
import csv
# Configure logs
logging.basicConfig(level=logging.DEBUG)

class segment:
  def __init__(self,pi,pf):
      self.pi=np.array(pi)
      self.pf=np.array(pf)
class RealTimePlotter:

  def __init__(self, id: str='RealPlotter', ip: str='192.168.1.109', cmd_port: int=5572, data_port:int=5573,
               hub_ip: str='192.168.1.109', hub_cmd_port: int=5555, hub_data_port: int=5556) -> None:
    self.id = id
    self.ip = ip
    self.cmd_port = cmd_port
    self.data_port = data_port
    self.hub_ip = hub_ip
    self.hub_cmd_port = hub_cmd_port
    self.hub_data_port = hub_data_port
    context = zmq.Context()
    self.control = context.socket(zmq.REQ)
    self.data = context.socket(zmq.PUB)
    self.data.bind(f'tcp://*:{data_port}')
    self.hub_data = context.socket(zmq.SUB)
    #for plot 
    self.ArenaLimitsReceived = False
    self.ArenaLimits = {}
    self.x=[None]*4
    self.y=[None]*4
    self.robotData = {}
    self.positions = {}
    self.robot_positions ={}
    self.plot_thread = threading.Thread(target=self.listen)
    self.exit_event = threading.Event()
    # Crear la figura y los ejes una vez al inicio
    self.fig, self.axs = plt.subplots(2, figsize=(8, 6))
    # Crear la figura y los ejes para los gráficos de las ruedas
    self.fig_wheel, self.axs_wheel = plt.subplots(2, figsize=(8, 6))
    
    # Crear la figura y los ejes para las posiciones de los robots
    self.fig_positions, self.axs_positions = plt.subplots(figsize=(8, 6))
  def _get_data_url(self):
    return f'tcp://{self.ip}:{self.data_port}'

  def _get_cmd_url(self):
    return f'tcp://{self.ip}:{self.cmd_port}'

  def _get_hub_data_url(self):
    return f'tcp://{self.hub_ip}:{self.hub_data_port}'

  def _get_hub_cmd_url(self):
    return f'tcp://{self.hub_ip}:{self.hub_cmd_port}'


  def register(self) -> None:
    '''Register to the hub'''
    print(f'Registering agent')
    self.control.connect(self._get_hub_cmd_url())
    self.control.send_json({
      'operation': 'hello',
      'source_id': self.id,
      'payload': {
        'url' : self._get_data_url()
      },
      'timestamp': 1000*time.time(),
    })
    response = self.control.recv_json()
    self.connected = response['result'] == 'ok'
    threading.Thread(target=self.start).start()#Thread listening to the hub

  def update_plot(self) -> None:
    plt.clf()
    num_robots = len(self.robotData)
    rows = num_robots//2 + num_robots%2
    cols = 2
    for i, (self.robot_name, data) in enumerate(self.robotData.items(), start=1):
        plt.subplot(rows, cols, i)
        plt.scatter(data['times'], data['w_left_values'], label='w_left')
        plt.scatter(data['times'], data['pwm_left_values'], label='pwm_left')
        plt.xlabel('Tiempo')
        plt.ylabel('Valor')
        plt.title(self.robot_name + " - Left Wheel")
        plt.legend()
        
        plt.subplot(rows, cols, i+num_robots)  # Subgráfica para la rueda derecha
        plt.scatter(data['times'], data['w_right_values'], label='w_right')
        plt.scatter(data['times'], data['pwm_right_values'], label='pwm_right')
        plt.xlabel('Tiempo')
        plt.ylabel('Valor')
        plt.title(self.robot_name + " - Right wheel")
        plt.legend()

    plt.tight_layout()
    plt.pause(0.01)  # Pausa para actualizar la gráfica
  def addLimits(self,limits):
        
    self.ArenaLimits = limits
    self.x[0]=-self.ArenaLimits["x1"]
    self.y[0]=-self.ArenaLimits["y1"]
    self.x[1]=-self.ArenaLimits["x2"]
    self.y[1]=-self.ArenaLimits["y2"]
    self.x[2]=-self.ArenaLimits["x3"]
    self.y[2]=-self.ArenaLimits["y3"]
    self.x[3]=-self.ArenaLimits["x4"]
    self.y[3]=-self.ArenaLimits["y4"]
    # self.x[0]=2.56
    # self.y[0]=2.29
    # self.x[1]=0.46
    # self.y[1]=2.29
    # self.x[2]=0.46
    # self.y[2]=0.89
    # self.x[3]=2.56
    # self.y[3]=0.89
    #self.segmentOfTheRectangle = self.computeSegments()
  def update_plot(self) -> None:
    # Limpiar los ejes antes de actualizar los datos
    self.axs[0].cla()
    self.axs[1].cla()
    
    for i, (robot_name, data) in enumerate(self.robotData.items(), start=1):
      
      self.axs_wheel[0].scatter(data['times'], data['w_left_values'], label='w_left')
      #self.axs[0].scatter(data['times'], data['pwm_left_values'], label='pwm_left')
      self.axs_wheel[0].set_xlabel('Tiempo')
      self.axs_wheel[0].set_ylabel('Valor')
      self.axs_wheel[0].set_title(robot_name + " - Left Wheel")
      self.axs_wheel[0].legend()
      self.axs_wheel[0].grid()

      self.axs_wheel[1].scatter(data['times'], data['w_right_values'], label='w_right')
      #self.axs[1].scatter(data['times'], data['pwm_right_values'], label='pwm_right')
      self.axs_wheel[1].set_xlabel('Tiempo')
      self.axs_wheel[1].set_ylabel('Valor')
      self.axs_wheel[1].set_title(robot_name + " - Right wheel")
      self.axs_wheel[1].legend()
      self.axs_wheel[1].grid()
      # plt.tight_layout()
      # plt.pause(0.01)  # Pausa para actualizar la gráfica
    self.axs_positions.cla()
   

    
    for robot_id, position_data in self.positions.items():
      # Convertir las coordenadas x e y a números flotantes
      x_position = float(position_data['x'])
      y_position = float(position_data['y'])
      
      # Agregar la posición a la lista correspondiente en el diccionario de posiciones
      if robot_id not in self.robot_positions:
          self.robot_positions[robot_id] = {'x': [], 'y': []}
      self.robot_positions[robot_id]['x'].append(x_position)
      self.robot_positions[robot_id]['y'].append(y_position)
        
    # Limpiar los ejes de las posiciones de los robots antes de actualizar los datos
    self.axs_positions.cla()
    
    # Actualizar los datos de las posiciones de los robots
    for robot_id, position_data in self.robot_positions.items():
        x_positions = position_data['x']
        y_positions = position_data['y']
        self.axs_positions.scatter(x_positions, y_positions, label=robot_id)
    
    # Configurar etiquetas y título
    self.axs_positions.set_xlabel('X')
    self.axs_positions.set_ylabel('Y')
    self.axs_positions.set_title('Posiciones de los Robots')
    self.axs_positions.legend()
    self.axs_positions.grid()
    # Trazar el rectángulo
    plt.xlim(self.x[0],self.x[2])
    plt.ylim(self.y[0],self.y[2])
    
    # Ajustar el diseño y mostrar la gráfica actualizada de las posiciones de los robots
    plt.tight_layout()
    plt.pause(0.01)  # Pausa para actualizar la gráfica

  def save_data(self):
    with open('robot_data.csv', 'w', newline='') as f:
      writer = csv.writer(f)
      writer.writerow(['Robot', 'Tiempo', 'w_left', 'w_right', 'pwm_left', 'pwm_right'])
      for robot_name, data in self.robotData.items():
        for i in range(len(data['times'])):
          writer.writerow([robot_name, data['times'][i], data['w_left_values'][i], 
                            data['w_right_values'][i], data['pwm_left_values'][i], 
                            data['pwm_right_values'][i]])
    with open('robot_positions.csv', 'w', newline='') as f:
      writer = csv.writer(f)
      writer.writerow(['Robot', 'X', 'Y'])
      for robot_id, position_data in self.robot_positions.items():
        for i in range(len(position_data['x'])):
          writer.writerow([robot_id, position_data['x'][i], position_data['y'][i]])
  
  def start(self):
     #take the arenaLimits
    while( self.ArenaLimitsReceived == False):
      self.send("localization/RobotariumData","")
      time.sleep(1)
    while True:
      choice = input("Presiona 'q' para salir o cualquier otra tecla para continuar: ")
      if choice.lower() == 'q':
        self.exit_event.set()
        self.save_data()
        break  
    

      
  def listen(self) -> None:
    '''Receive data from other agents'''
    logging.debug(f'Connecting to hub at {self._get_hub_data_url()}')
    self.hub_data.connect(self._get_hub_data_url())

    logging.debug('Subscribing to data')
    self.hub_data.setsockopt(zmq.SUBSCRIBE, b'data')

    logging.debug('Subscribing to control')
    self.hub_data.setsockopt(zmq.SUBSCRIBE, b'control/RealTimePlotter')

    # self.hub_data.setsockopt_string(zmq.SUBSCRIBE, f'{self.id}/control')
    self.addLimits("json.loads(message)")
    self.update_plot()
    while not self.exit_event.is_set():
      topic = self.hub_data.recv_string()
      message = self.hub_data.recv_string()
      _, agent, topic = topic.split('/')
      if topic == 'telemetry':
        if agent not in self.robotData:
          self.robotData[agent] = {'times': [], 'w_left_values': [], 'w_right_values': [], 'pwm_left_values': [], 'pwm_right_values': []}
        self.ArenaLimitsReceived 
        current_time = self.robotData[agent]['times'][-1] if self.robotData[agent]['times'] else 0
        self.robotData[agent]['times'].append(current_time + 100)
        self.robotData[agent]['w_left_values'].append(json.loads(message)['w_left'])
        self.robotData[agent]['w_right_values'].append(json.loads(message)['w_right'])
        self.robotData[agent]['pwm_left_values'].append(json.loads(message)['pwm_left'])
        self.robotData[agent]['pwm_right_values'].append(json.loads(message)['pwm_right'])
      elif topic == 'position':
        self.positions[agent] = json.loads(message)
      elif topic == "ArenaSize":
        self.ArenaLimitsReceived = True

        self.addLimits(json.loads(message))

      #update plot
    

     
  def send(self, topic: str, data: dict) -> None:
    '''Send data to a topic'''
    self.data.send_string(topic, flags=zmq.SNDMORE)
    self.data.send_json(data)

if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = RealTimePlotter()
    agent.register()
    agent.listen()
