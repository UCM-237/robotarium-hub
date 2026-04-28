import json
import numpy as np
from agent import Agent
import logging
import math
import csv
from datetime import datetime
import time
#necesario para recibir por mqtt
import paho.mqtt.client as mqtt
BROKER = "192.168.10.1"
PUERTO = 1883


class BouncerRobot:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.boundaries=[0.0,0.0,0.0,0.0]
        self.margin = 0.1
        self.speed=10.0
        self.direction=[0.707, 0.707]
        self.robot_id=6
        self.pos=[0.0,0.0,0.0]
        self.angular_speed=1.0
        self.safety_distance = 0.2 
        self.is_turning=False
        # --- Configuración del Logger ---
        self.log_file = f"robot_{self.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_logger()
        

    def init_logger(self):
        with open(self.log_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Cabecera con todos los datos que pediste
            writer.writerow([
                "timestamp", "x", "y", "theta", 
                "x_min", "x_max", "y_min", "y_max", 
                "dist_to_wall", "decision_v", "decision_w"
            ])
    def log_data(self, x, y, theta, dist, v, w):
        with open(self.log_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                time.time(), x, y, theta,
                self.boundaries[0], self.boundaries[1], 
                self.boundaries[2], self.boundaries[3],
                round(dist, 3), v, w
            ])


    def connect(self) -> None:
        '''Establish a connection with the hardware'''

    def on_data(self, topic: str, message: str) -> None:
        '''Handle incoming data'''
        # 1. Recibir límites del tatami (vienen del arena_agent)
        if topic == "arena/boundaries":
            try:
                raw_data = json.loads(message)
                if isinstance(raw_data, str):
                    raw_data = json.loads(raw_data)
                
                puntos = raw_data["points"]
            
                all_x = [p["x"] for p in puntos]
                all_y = [p["y"] for p in puntos]
            
                # 4. Guardar los valores extremos para la lógica de rebote
                self.x_min = min(all_x)
                self.x_max = max(all_x)
                self.y_min = min(all_y)
                self.y_max = max(all_y)
                self.boundaries = [self.x_min,self.x_max,self.y_min,self.y_max]
            except Exception as e:
                print(f"Error al decodificar: {e}")
        # 2. Recibir posición del robot (vienen del pos_agent)
        elif topic == "6/pos":
            try:
                raw_data= json.loads(message)
                if isinstance(raw_data, str):
                        raw_data = json.loads(raw_data)
                
                self.pos[0]=float(raw_data.get('x'))
                self.pos[1]=float(raw_data.get('y'))
                self.pos[2]=float(raw_data.get('yaw'))
                self.check_collision_and_move()
            except Exception as e:
                print(f"Error al descodificar: {e}")


  
        
    def get_distance_to_wall(self, x, y, theta):
            # Distancias a las 4 paredes (asumiendo tatami rectangular)
            # Basado en: x + d*cos(theta) = x_limit  =>  d = (x_limit - x) / cos(theta)
            
            distances = []
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            # Paredes verticales (X min y max)
            if abs(cos_t) > 1e-6:
                distances.append((self.boundaries[0] - x) / cos_t) # x_min
                distances.append((self.boundaries[1] - x) / cos_t) # x_max
            
            # Paredes horizontales (Y min y max)
            if abs(sin_t) > 1e-6:
                distances.append((self.boundaries[2] - y) / sin_t) # y_min
                distances.append((self.boundaries[3] - y) / sin_t) # y_max

            # Solo nos interesan distancias positivas (hacia adelante)
            logging.info(f"Distances to borders: {distances}")
            positives = [d for d in distances if d > 0]
            return min(positives) if positives else float('inf')

    def check_collision_and_move(self):
        x, y, theta = self.pos
        dist = self.get_distance_to_wall(x, y, theta)
        v=0.0
        w=0.0

        if dist < self.safety_distance and not self.is_turning:
            # Iniciamos maniobra de giro: v=0, w=velocidad_giro
            self.is_turning = True
            v=0.0
            w=1.5
            
        elif self.is_turning and dist > self.safety_distance * 1.5:
            # Ya estamos apuntando a sitio seguro
            self.is_turning = False
            v=self.speed
            w=0.0
        elif not self.is_turning:
            v=self.speed
            w=0.0

        self.log_data(x, y, theta, dist, v, w)

        self.send_move(v,w)

    def send_move(self, v, w):
        bouncer_agent.send(f"agent/{self.robot_id}/move", {'v': v, 'w': w})

# a partir de aqui es todo de recibir
#cuando conecta
def on_connect(client,userdata,flags,rc):
   print("conectado al broker")
   #client.subscribe("#")
   client.subscribe("agent/6/velocity")   
   client.subscribe("agent/6/odon")
   client.subscribe("arena/boundaries")     
   client.subscribe("6/pos")      
   #client.subscribe("agent/5/wheel")         

#cuando llega el mensaje
def on_message(client,userdata, msg):
    print("topic:", msg.topic)
    print("Mensaje:", msg.payload.decode()) 
    print("------")
      
# Configuración del Agente
if __name__ == "__main__":
   # Configuración del Agente
    bouncer_agent = Agent(
      device_class=BouncerRobot,
      id='Bouncer Robot',
      ip='192.168.10.1',
      data_port = 5562,
      hub_ip='192.168.10.1'
    )
    
    #MQTT_agent.register()
    logging.info(f'Agent {bouncer_agent.id} is listening')

    # Configuración MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PUERTO, 60)
    client.loop_start()
    logging.info(f"Agent {bouncer_agent.id} en marcha")

    