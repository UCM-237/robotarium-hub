# -*- coding: UTF-8 -*-
import sys
import tty
import termios
import select
import time
from agent import Agent
import json
import numpy as np
from agent import Agent
import logging
import math
import csv
from datetime import datetime
#necesario para recibir por mqtt
import paho.mqtt.client as mqtt
BROKER = "192.168.10.1"
PUERTO = 1883


class GetKey:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

# Creamos una clase nueva que EXTIEUNDE a la que ya funciona
class Teleoperator:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.v =0.0
        self.w =0.0
        self.robot_id=6
        # --- Configuración del Logger ---
        self.log_file = f"robot_{self.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_logger()
        self.gk = GetKey()

    def init_logger(self):
        with open(self.log_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Cabecera con todos los datos que pediste
            writer.writerow([
                "timestamp", "v", "w"
            ])
    def log_data(self, x, y, theta, dist, v, w):
        with open(self.log_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                time.time(), self.v, self.w
            ])


    def connect(self) -> None:
        '''Establish a connection with the hardware'''

    def on_data(self, topic: str, message: str) -> None:
        '''Handle incoming data'''
    def run_teleop_loop(self):
        print("\n" + "="*40)
        print("   TELEOP ROBOTARIUM (PI_AGENT_LIMITS)")
        print("="*40)
        print(" W/S +- vlineal | A/D: Angular | G/H: Angulo giro | Espacio: STOP")
        print(" Q: Salir")
        
        try:
            while True:
                key = self.gk.get_key()
                
                if key == 'w': self.v += 0.5
                elif key == 's': self.v -= 0.5
                elif key == 'a': self.w -= 0.1
                elif key == 'd': self.w += 0.1
                elif key == 'g': self.ang +=5
                elif key == 'h': self.ang -=5
                elif key == ' ':
                    self.v, self.w = 0.0, 0.0
                   
                elif key == 'q':
                    break

                if key in ['w', 's', 'a','d',' ']:
                    # Usamos el método move_robot que ya está definido en pi_agent_limits.py
                    # Ese método ya hace el empaquetado y envío al Arduino
                    print(f"\rV: {self.v:5.2f} | W: {self.w:5.2f} ", end='', flush=True)
                    vl=self.v-(13.1/2.0)*self.w
                    vr=2*self.v-vl                    
                    wl=vl/3.35
                    wr=vr/3.35
                    print(f"\r wr={wr}, wl={wl} (rad/s)",end='',flush=True)
                    self.send_move(wl,wr)
                elif key in ['g','h']:
                    print(f"Ang. giro: {self.ang} (grad)",end='',flush=True)
                    self.send_move_ang(self.ang)    

                time.sleep(0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.gk.settings)
    def send_move(self, v, w):
        teleop_agent.send(f"agent/{self.robot_id}/move", {'v': v, 'w': w})   
    def send_move_ang(self, ang):
        teleop_agent.send(f"agent/{self.robot_id}/turn", {'ang': ang})   

if __name__ == "__main__":
      # Configuración del Agente
    teleop_agent= Agent(
      device_class=Teleoperator,
      id='TeleopAgent',
      ip='192.168.10.1',
      data_port = 5566,
      hub_ip='192.168.10.1'
    )
    
    #MQTT_agent.register()
    logging.info(f'Agent {teleop_agent.id} is listening')

    # Configuración MQTT
    client = mqtt.Client()
    client.connect(BROKER, PUERTO, 60)
    client.loop_start()
    logging.info(f"Agent {teleop_agent.id} en marcha")
    teleop_agent.device.run_teleop_loop()
