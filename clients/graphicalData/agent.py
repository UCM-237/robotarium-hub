# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import time
import zmq

import matplotlib.pyplot as plt
import numpy as np
import threading
import queue
import time
import json
import pickle
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import csv
# Configure logs
logging.basicConfig(level=logging.DEBUG)


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
    self.robotData = {}
    self.plot_thread = threading.Thread(target=self.listen)
    self.exit_event = threading.Event()
    # Crear la figura y los ejes una vez al inicio
    self.fig, self.axs = plt.subplots(2, figsize=(8, 6))

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

  # def update_plot(self) -> None:
  #   plt.clf()
  #   num_robots = len(self.robotData)
  #   rows = num_robots//2 + num_robots%2
  #   cols = 2
  #   for i, (self.robot_name, data) in enumerate(self.robotData.items(), start=1):
  #       plt.subplot(rows, cols, i)
  #       plt.scatter(data['times'], data['w_left_values'], label='w_left')
  #       plt.scatter(data['times'], data['pwm_left_values'], label='pwm_left')
  #       plt.xlabel('Tiempo')
  #       plt.ylabel('Valor')
  #       plt.title(self.robot_name + " - Left Wheel")
  #       plt.legend()
        
  #       plt.subplot(rows, cols, i+num_robots)  # Subgráfica para la rueda derecha
  #       plt.scatter(data['times'], data['w_right_values'], label='w_right')
  #       plt.scatter(data['times'], data['pwm_right_values'], label='pwm_right')
  #       plt.xlabel('Tiempo')
  #       plt.ylabel('Valor')
  #       plt.title(self.robot_name + " - Right wheel")
  #       plt.legend()

  #   plt.tight_layout()
  #   plt.pause(0.01)  # Pausa para actualizar la gráfica
  def update_plot(self) -> None:
    # Limpiar los ejes antes de actualizar los datos
    self.axs[0].cla()
    self.axs[1].cla()
    
    for i, (robot_name, data) in enumerate(self.robotData.items(), start=1):
      
      self.axs[0].scatter(data['times'], data['w_left_values'], label='w_left')
      #self.axs[0].scatter(data['times'], data['pwm_left_values'], label='pwm_left')
      self.axs[0].set_xlabel('Tiempo')
      self.axs[0].set_ylabel('Valor')
      self.axs[0].set_title(robot_name + " - Left Wheel")
      self.axs[0].legend()
      self.axs[0].grid()

      self.axs[1].scatter(data['times'], data['w_right_values'], label='w_right')
      #self.axs[1].scatter(data['times'], data['pwm_right_values'], label='pwm_right')
      self.axs[1].set_xlabel('Tiempo')
      self.axs[1].set_ylabel('Valor')
      self.axs[1].set_title(robot_name + " - Right wheel")
      self.axs[1].legend()
      self.axs[1].grid()
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
  
  def start(self):
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
    while not self.exit_event.is_set():
      topic = self.hub_data.recv_string()
      message = self.hub_data.recv_string()
      _, agent, topic = topic.split('/')
      if agent not in self.robotData:
        self.robotData[agent] = {'times': [], 'w_left_values': [], 'w_right_values': [], 'pwm_left_values': [], 'pwm_right_values': []}
      
      current_time = self.robotData[agent]['times'][-1] if self.robotData[agent]['times'] else 0
      self.robotData[agent]['times'].append(current_time + 100)
      self.robotData[agent]['w_left_values'].append(json.loads(message)['w_left'])
      self.robotData[agent]['w_right_values'].append(json.loads(message)['w_right'])
      self.robotData[agent]['pwm_left_values'].append(json.loads(message)['pwm_left'])
      self.robotData[agent]['pwm_right_values'].append(json.loads(message)['pwm_right'])
      self.update_plot()
      
     

if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = RealTimePlotter()
    agent.register()
    agent.listen()