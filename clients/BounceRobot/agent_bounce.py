import json
import numpy as np
from agent import Agent
import logging
#necesario para recibir por mqtt
import paho.mqtt.client as mqtt
BROKER = "192.168.10.1"
PUERTO = 1883


class BouncerRobot:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.boundaries=[0.0,0.0,0.0,0.0]
        self.margin = 0.2
        self.speed=10
        self.direction=[0.707, 0.707]
        self.robot_id=5
        self.pos=[0.0,0.0,0.0]

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
        elif topic == "5/pos":
            try:
                raw_data= json.loads(message)
                if isinstance(raw_data, str):
                        raw_data = json.loads(raw_data)
                
                self.pos=raw_data
                print(self.pos)
                self.update_behavior()
            except Exception as e:
                print(f"Error al descodificar: {e}")


  
        
    def update_behavior(self):
        if not self.boundaries:
            return

        # Extraer límites (asumiendo rectángulo ordenado: 0:SI, 1:SD, 2:ID, 3:II)
        # Usamos los valores extremos para simplificar el rebote
        x_min = self.boundaries[0]
        x_max = self.boundaries[1]
        y_min = self.boundaries[0]
        y_max = self.boundaries[2]

        curr_x = self.pos['x']
        curr_y = self.pos['y']

        # LOGICA DE REBOTE
        rebound = False

        # Rebote en X (Paredes laterales)
        if curr_x <= (x_min + self.margin) and self.direction[0] < 0:
            self.direction[0] *= -1
            rebound = True
        elif curr_x >= (x_max - self.margin) and self.direction[0] > 0:
            self.direction[0] *= -1
            rebound = True

        # Rebote en Y (Paredes fondo/frente)
        if curr_y <= (y_min + self.margin) and self.direction[1] < 0:
            self.direction[1] *= -1
            rebound = True
        elif curr_y >= (y_max - self.margin) and self.direction[1] > 0:
            self.direction[1] *= -1
            rebound = True

        if rebound:
            print(f"[BOUNCE] Robot {self.robot_id} rebotó en pared. Nueva dirección: {self.direction}")

        # Enviar comando al robot
        # Calculamos v_x y v_y basados en la dirección y velocidad constante
        vx = self.direction[0] * self.speed
        vy = self.direction[1] * self.speed
        
        cmd = {
            "vx": round(float(vx), 2),
            "vy": round(float(vy), 2)
        }
        bouncer_agent.send("5/move", json.dumps(cmd))
        logging.debug(f"Comando enviado -> v: {vx:.1f} | w: {vy:.1f}")

# a partir de aqui es todo de recibir
#cuando conecta
def on_connect(client,userdata,flags,rc):
   print("conectado al broker")
   #client.subscribe("#")
   client.subscribe("agent/5/velocity")   
   client.subscribe("agent/5/odon")
   client.subscribe("arena/boundaries")     
   client.subscribe("5/pos")      
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

    