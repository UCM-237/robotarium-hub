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
import threading
BROKER = "192.168.10.1"
PUERTO = 1883


class BouncerRobot:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.boundaries=[0.0,0.0,0.0,0.0]
        self.margin = 0.1
        self.speed=6.0
        self.v=25.0
        self.w=1.5
        self.direction=[0.707, 0.707]
        self.robot_id=6
        self.pos=[0.0,0.0,0.0]
        self.angular_speed=1.0
        self.safety_distance = 10.0 
        self.is_turning =False
        # --- Configuración del Logger ---
        self.log_file = f"robot_{self.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_logger()
        self.last_pos_time = 0.0
        
        self.estimate = [0.0, 0.0, 0.0] # [xe, ye, thetae] - Estima por odometría
        
        # Parámetros físicos del robot (deben coincidir con robot.h)
        self.wheel_radius = 3.35 # cm
        self.robot_width = 14.5  # cm (distancia entre ruedas)
        
        self.last_odom_time = time.time()
        self.last_vision_time = time.time()
        self.status = "INICIALIZADO"

    def init_logger(self):
        with open(self.log_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Cabecera con todos los datos que pediste
            writer.writerow([
                "timestamp", "x", "y", "theta", "x_estimate", "y_estimate", "theta_estimate","status",
                "x_min", "x_max", "y_min", "y_max", 
                "dist_to_wall", "decision_v", "decision_w"
            ])
            
    def log_data(self, x, y, theta, dist, v, w):
        with open(self.log_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                time.time(), x, y, theta,
                self.estimate[0], self.estimate[1], self.estimate[2], self.status,
                self.boundaries[0], self.boundaries[1], self.boundaries[2], self.boundaries[3],
                round(dist, 3), v, w
            ])
    def connect(self) -> None:
        '''Establish a connection with the hardware'''

    def on_pos_received(self, x, y, theta):
        """Actualiza la posición real y sincroniza la estima."""
        self.pos = [x, y, theta]
        self.estimate = [x, y, theta] # Sincronización: la cámara manda
        self.last_vision_time = time.time()
        logging.info(f"Posición actualizada por visión: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

    def on_odom_received(self, wl, wr):
        """Calcula el movimiento basado en encoders (Cinemática Diferencial)."""
        current_time = time.time()
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        # 1. Velocidades lineales de cada rueda (cm/s)
        v_left = wl * self.wheel_radius
        v_right = wr * self.wheel_radius

        # 2. Velocidad lineal y angular del centro del robot
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.robot_width

        # 3. Actualizar la estima (Integración numérica)
        # Usamos el ángulo actual de la estima
        theta = self.estimate[2]
        
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = w * dt

        self.estimate[0] += dx
        self.estimate[1] += dy
        self.estimate[2] += dtheta # Normalizar si es necesario
        logging.info(f"Actualización por odometría: Δx={dx:.2f}, Δy={dy:.2f}, Δθ={dtheta:.2f}")
        self.last_odom_time = current_time


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
                print(raw_data)
                if isinstance(raw_data, str):
                        raw_data = json.loads(raw_data)
                
                self.pos[0]=float(raw_data.get('x'))
                self.pos[1]=float(raw_data.get('y'))
                self.pos[2]=float(raw_data.get('yaw'))
                self.on_pos_received(self.pos[0], self.pos[1], self.pos[2])
                current_time = time.time()
                sent_time = raw_data.get("timestamp")
   
                latency = (current_time - sent_time) * 1000 # Latencia en ms
    
                # Calcular frecuencia (Delta tiempo entre este mensaje y el anterior)
                if hasattr(self, 'last_pos_time'):
                    freq = 1.0 / (current_time - self.last_pos_time)
                    logging.info(f"Frecuencia: {freq:.2f} Hz | Latencia Red/Proc: {latency:.2f} ms")
    
                self.last_pos_time = current_time
                self.check_collision_and_move()
                
            except Exception as e:
                print(f"Error al descodificar: {e}")
        elif topic == "agent/6/wheel":
            try:
                raw_data = json.loads(message)
                if isinstance(raw_data, str):
                    raw_data = json.loads(raw_data)
                
                wl = float(raw_data.get('WLeft'))
                wr = float(raw_data.get('WRight'))
                self.on_odom_received(wl, wr)
            except Exception as e:
                print(f"Error al decodificar odometría: {e}")
    
    def run(self):
        """Bucle de control independiente que corre a ~20Hz"""
        while True:
            ahora = time.time()
            
            # 1. VERIFICACIÓN DE SEGURIDAD (WATCHDOG)
            # Si hace más de 1.5 segundos que no sabemos nada del robot...
            time_since_vision = ahora - self.last_pos_time
            time_since_odom = ahora - self.last_odom_time
            
            if time_since_vision > 1.5 and time_since_odom > 1.5:
                logging.warning("SISTEMA DESCONECTADO: Parando robot por seguridad")
                self.send_move(0, 0)
            else:
                # 2. EJECUCIÓN DE LA LÓGICA
                # pos_logic ahora decidirá qué posición usar
                self.check_collision_and_move()
            
            time.sleep(0.05) # 20 Hz

  
        
    def get_distance_to_wall(self, x, y, theta):
        # 1. Límites actuales (centímetros)
        x_min, x_max = self.boundaries[0], self.boundaries[1]
        y_min, y_max = self.boundaries[2], self.boundaries[3]

        # 2. Distancias Euclidianas "puras" (¿A cuánto estoy de las bandas?)
        d_left = x - x_min
        d_right = x_max - x
        d_top = y - y_min
        d_bottom = y_max - y
        logging.info(f"Distancias a paredes: Left: {d_left:.2f}, Right: {d_right:.2f}, Top: {d_top:.2f}, Bottom: {d_bottom:.2f}")       
                    
        return [d_left,d_right,d_top,d_bottom]
    
    def check_collision_and_move(self):
        ahora = time.time()
        
        # DECISIÓN DE POSICIÓN
        # Prioridad 1: Visión (si es reciente < 0.5s)
        if (ahora - self.last_pos_time) < 0.5:
            x, y, theta = self.pos
            self.status = "VISION"
        # Prioridad 2: Estima por odometría
        else:
            x, y, theta = self.estimate
            self.status = "ESTIMA"
        # 2. Obtener distancias a paredes
        [d_left, d_right, d_top,d_bottom] = self.get_distance_to_wall(x, y, theta)
        
        # 3. Dirección del movimiento
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # 4. Lógica de peligro:
        # Solo consideramos que una distancia es "peligrosa" si el robot se dirige hacia ella
        danger_distances = []
            
        if cos_t < 0: danger_distances.append(d_left)   # Se mueve a la izquierda
        if cos_t > 0: danger_distances.append(d_right)  # Se mueve a la derecha
        if sin_t < 0: danger_distances.append(d_top)    # Se mueve hacia arriba
        if sin_t > 1e-6: danger_distances.append(d_bottom) # Se mueve hacia abajo (tu eje Y)

        logging.info(f"Distancias reales a paredes de interés: {danger_distances}")
        if danger_distances is  None and not self.is_turning:
           v=self.speed
           w=0.0
           self.is_turning=False
           logging.info("Zona segura")
        elif np.min(danger_distances)<=self.safety_distance and not self.is_turning:
           self.is_turning=True
           v=0.0
           w=self.w
           logging.info("Iniciando giro")
        elif np.min(danger_distances)<=self.safety_distance and self.is_turning:
           v=0.0
           w=self.w
           logging.info("Continuando giro")
        elif np.min(danger_distances)>self.safety_distance:
           self.is_turning=False
           v=self.v
           w=0.0
           logging.info("Zona segura")
        else:
           v=0.0
           w=0.0
           logging.info("Caso indeterminado")
        # Si d < 0, significa que YA se salió. Devolvemos 0 para forzar rebote inmediato
        
        self.log_data(x, y, theta, np.min(danger_distances), v, w)
        vl=v-(13.1/2.0)*w
        vr=2*v-vl                    
        wl=vl/3.35
        wr=vr/3.35
        self.send_move(wl,wr)
        logging.info(f"Enviada v: {wl} w: {wr}")


    def send_move(self, v, w):
        bouncer_agent.send(f"agent/{self.robot_id}/move", {'v': v, 'w': w})

# a partir de aqui es todo de recibir
#cuando conecta
def on_connect(client,userdata,flags,rc):
   print("conectado al broker")
   #client.subscribe("#")
   #client.subscribe("agent/6/velocity")   
   #client.subscribe("agent/6/odon")
   client.subscribe("arena/boundaries")     
   client.subscribe("agent/6/pos")      
   client.subscribe("agent/6/wheel")         

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

    t = threading.Thread(target=bouncer_agent.device.run)
    t.daemon = True # Se cierra cuando cierres el programa principal
    t.start()
    
    # El agente se queda escuchando MQTT
    bouncer_agent.listen()
    
