# -*- coding: UTF-8 -*-
import sys
import tty
import termios
import select
import time
from agent import Agent
from agent import Agent
import logging
import csv
from datetime import datetime
#necesario para recibir por mqtt
import paho.mqtt.client as mqtt
BROKER = "192.168.10.1"
PUERTO = 1883
import argparse

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
        self.ang=0
        # --- Configuración del Logger ---
        self.log_file = f"logs/robot_{self.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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
                    print(f"\r V: {self.v:5.2f} | W: {self.w:5.2f} ", end='', flush=True)
                    self.send_move(self.v,self.w)
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
    parser = argparse.ArgumentParser(
        description="Agente Remote Control para el Robotarium"
    )
    
    # Parámetro obligatorio posicional (o puedes hacerlo opcional con '--id')
    parser.add_argument(
        'robot_id', 
        type=int, 
        help="ID numérico del robot a controlar (ej. 5, 8)"
    )
    
    # Parámetro opcional con valor por defecto
    parser.add_argument(
        '-p', '--port', 
        type=int, 
        default=5572,  # Cambia esto por el puerto por defecto real de tu arquitectura ZMQ
        help="Puerto de datos (data_port) para la conexión ZeroMQ. Por defecto: 5572"
    )
    
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, 
        format=f"[Robot {args.robot_id}] %(asctime)s - %(levelname)s - %(message)s"
    )

    logging.info(f"Iniciando configuración... ID: {args.robot_id} | Puerto ZMQ: {args.port}")

    
    # Inicializamos el Agente pasándole el ID como string y el puerto dinámico
    teleop_agent = Agent(
        device_class=Teleoperator,
        id=f'TeleopAgent_{args.robot_id}',  # ID único para el agente de red
        ip='192.168.10.1',
        data_port=args.port                 # Inyección del puerto dinámico
    ) 
    # 3. Sincronización del robot_id dentro de la clase interna Teleoperator
    teleop_agent.device.robot_id = args.robot_id
    teleop_agent.device.log_file = f"robot_{args.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    teleop_agent.device.init_logger() 
    #MQTT_agent.register()
    logging.info(f'Agent {teleop_agent.id} is listening')

    # Configuración MQTT
    client = mqtt.Client()
    client.connect(BROKER, PUERTO, 60)
    client.loop_start()
    logging.info(f"Agent {teleop_agent.id} en marcha")
    teleop_agent.device.run_teleop_loop()
