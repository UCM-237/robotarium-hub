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
from queue import Queue # Para comunicar hilos de forma segura
from enum import Enum
from logger_config import setup_logger
import logging

BROKER = "192.168.10.1"
PUERTO = 1883

'''TODO: Implementar lógica de rebote basada en distancias a paredes. El robot debería "rebotar" (girar) cuando se acerque demasiado a una pared, y luego volver a avanzar.
Que lo haga en la dirección opuesta a la pared más cercana. Para esto, el robot debe calcular su distancia a cada pared (usando su posición y los límites del tatami) y decidir hacia dónde girar. Además, implementar un sistema de "estados" (FSM) para manejar las transiciones entre avanzar, girar y parar. Por ejemplo:
- Estado "Avanza": El robot se mueve hacia adelante. Si detecta que se acerca a una pared (distancia < umbral), cambia al estado "Gira".
- Estado "Gira": El robot gira en la dirección opuesta a la pared más cercana durante un tiempo determinado o hasta que alcance una distancia segura. Luego vuelve al estado "Avanza".
- Estado "Para": Si el robot detecta que está demasiado cerca de una pared (distancia < umbral crítico), se detiene completamente para evitar colisiones. Permanece en este estado hasta que la distancia vuelva a ser segura, momento en el cual puede volver a "Avanza".
TODO: Mejorar la lógica de decisión para considerar no solo la distancia a las paredes, sino también la dirección del movimiento. Por ejemplo, si el robot se está moviendo hacia una pared, esa pared debería tener más peso en la decisión de rebote que una pared que está detrás del robot. Esto se puede lograr calculando el ángulo entre la dirección del movimiento y la dirección hacia cada pared, y ajustando el umbral de distancia en función de este ángulo.
TODO: Mejorar la fusión de datos entre la posición por visión y la estima por odometría. En lugar de simplemente priorizar la visión cuando está disponible, se podría implementar un filtro de Kalman o un sistema de ponderación que combine ambas fuentes de información para obtener una estimación más robusta de la posición del robot. Esto ayudaría a mitigar los efectos de la latencia en la visión y los errores acumulativos en la odometría, proporcionando una base más sólida para la lógica de rebote y navegación. 
'''


class BillarState(Enum):
    AVANZA = 1
    PARANDO_PARA_GIRAR = 2
    GIRANDO = 3
    ESPERANDO_GIRO = 4
    
class BouncerRobot:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.boundaries=[0,419,0,140] #Lo inicializo asi por si acaso no recibe los limites
        self.x_min=0
        self.x_max=419
        self.y_max=140
        self.y_min=0
        self.margin = 20.0
        self.speed = 20.0
        self.fsm = BillarState.AVANZA
        self.fsm_last=BillarState.AVANZA
        self.danger_distance = 20.0
        self.last_wall_hit=None
        self.v=10.0
        self.w=3.0
        self.direction=[0.707, 0.707]
        self.robot_id=6
        self.pos=[0.0,0.0,0.0]
        self.angular_speed=1.0
        self.safety_distance = 40.0 
        self.t_retrocediendo=0
        self.control_rate=0.05 #ms
        self.last_time=0
        self.command_queue = Queue() # Cola para enviar comandos al agente
        self.giro_terminado=False
        # --- Configuración del Logger ---
        self.log_file = f"robot_{self.robot_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.last_pos_time = 0.0
        self.stop_duration = 0.5  # Tiempo de parada en segundos
        self.stop_start_time = 0
        self.retrocede_duration = 2.0
        self.retrocede_start_time = 0
        self.estimate = [0.0, 0.0, 0.0] # [xe, ye, thetae] - Estima por odometría
        self.target_theta=0.0
        # Parámetros físicos del robot (deben coincidir con robot.h)
        self.wheel_radius = 3.35 # cm
        self.robot_width = 14.5  # cm (distancia entre ruedas)
        self.angle_limit= 0.8        
        self.last_odom_time = time.time()
        self.last_vision_time = time.time()
        self.status = "ESPERANDO POSICIÓN INICIAL"
    
   
    def connect(self) -> None:
        '''Establish a connection with the hardware'''

    def on_pos_received(self, x, y, theta):
        """Actualiza la posición real y sincroniza la estima."""
        self.pos = [x, y, theta]
        self.estimate = [x, y, theta] # Sincronización: la cámara manda
        self.last_vision_time = time.time()
        logger.debug(f"Posición actualizada por visión: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

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
        #logger.info(f"Actualización por odometría: Δx={dx:.2f}, Δy={dy:.2f}, Δθ={dtheta:.2f}")
        self.last_odom_time = current_time


    def on_data(self, topic: str, message: str) -> None:
        logger.debug(f"Incoming data. Topic {topic}, mensaje {message}")
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
                logger.error(f"Error al decodificar: {e}")
        # 2. Recibir posición del robot (vienen del pos_agent)
        elif topic == f"{self.robot_id}/pos":
            logger.info(f"Message {message} received on topic {topic}")
            self.status = "INICIALIZADO"
            try:
                # Validar que no estemos recibiendo un string plano o un tópico desalineado
                if not message.startswith('{'):
                    logger.error(f"Mensaje malformado o trama desalineada detectada: {message}")
                    return
                
                raw_data = json.loads(message)
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
                    logger.debug(f"Frecuencia: {freq:.2f} Hz | Latencia Red/Proc: {latency:.2f} ms")
    
                self.last_pos_time = current_time
                
                
            except Exception as e:
                logger.error(f"Error al descodificar: {e}")
        elif topic == f"agent/{self.robot_id}/wheel":
            
            try:
                raw_data = json.loads(message)
                if isinstance(raw_data, str):
                    
                    raw_data = json.loads(raw_data)
                
                wl = float(raw_data.get('Wleft'))
                wr = float(raw_data.get('Wright'))
                self.on_odom_received(wl, wr)
            except Exception as e:
                logger.error(f"Error al decodificar odometría: {e}")
        elif topic == f"agent/{self.robot_id}/feedback":
            try:
                raw_data = json.loads(message)
                if isinstance(raw_data, str):
                    raw_data = json.loads(raw_data)
                status=raw_data.get("status")                
                op=raw_data.get("op")
                if status == "done" and op == "turn":
                    logger.info("¡Confirmación recibida desde Arduino! Giro terminado exitosamente.")
                    self.giro_terminado = True
            except Exception as e:
                logger.error(f"Error al decodificar feedback: {e}")

            

  
        
    def get_distance_to_wall(self, x, y, theta):
        # 1. Límites del tatami (cm)
        x_min, x_max = self.boundaries[0], self.boundaries[1]
        y_min, y_max = self.boundaries[2], self.boundaries[3]
        # Vector unitario de dirección del robot
        # 2. Vector de dirección del movimiento según la convención de tu código
        vy = math.cos(theta)
        vx = -math.sin(theta)
        logger.critical(f"Velocidades proyectadas vx: {vx}, vy: {vy}")
        # 3. Calcular la distancia proyectada en la trayectoria para el eje X e Y
        dist_x = float('inf')
        if vx > 0:
            dist_x = (x_max - x) / vx  # Pared derecha
            pared_l="derecha"
        elif vx < 0:
            dist_x = (x) / -vx  # Pared izquierda
            pared_l="izquierda"

        dist_y = float('inf')
        if vy > 0:
            dist_y = (y) / vy  # Pared de arriba
            pared_f="arriba"
            # Nota: Si tu eje Y crece hacia abajo, dist_y = (y_max - y) / vy es correcto.
            # Si el eje Y crece hacia arriba, ajusta los signos según corresponda.
        elif vy < 0:
            dist_y = (y_max - y) / -vy  # Pared de abajo
            pared_f="abajo"
        # 4. La distancia real a la pared que impactará primero en su trayectoria
        distance_to_target_wall = min(dist_x, dist_y)
        if dist_x<dist_y:
            pared=pared_l
        else:
            pared=pared_f
        logger.critical(f"Distancia a la pared {pared} en trayectoria: {distance_to_target_wall:.2f} cm")
        return distance_to_target_wall,pared
    
    def run(self):
        """Bucle principal de control a ~20Hz"""
        while True:
            ahora = time.time()
            if (ahora - self.last_time) >= self.control_rate:
                
                self.last_time = ahora
                time_since_vision = ahora - self.last_pos_time
                time_since_odom = ahora - self.last_odom_time
                
                if self.status=="ESPERANDO POSICIÓN INICIAL":
                    logger.warning("Esperando posición inicial... Aún no se han recibido datos de visión.")
                    self.command_queue.put({'v': 0.0, 'w': 0.0})
                    continue
                else:
                    if time_since_vision > 2.5 and time_since_odom > 2.5:
                        logger.warning("SISTEMA DESCONECTADO: Parando robot por seguridad")
                        self.command_queue.put({'v': 0.0, 'w': 0.0})
                        continue
                    else:
                        x, y, theta = self.pos
                        dist_frontal, pared = self.get_distance_to_wall(x, y, theta)
                        
                        logger.warning(f"Pos: ({x:.1f}, {y:.1f}) | Yaw: {math.degrees(theta):.1f}° | Dist Frontal: {dist_frontal:.1f} cm a la pared {pared}")
                        
                        # Máquina de Estados Finitos (FSM)
                        if self.fsm == BillarState.AVANZA:
                            if dist_frontal < self.danger_distance:
                                logger.warning(f"¡Obstáculo detectado a {dist_frontal:.1f} cm! Parando robot para iniciar giro.")
                                self.fsm = BillarState.PARANDO_PARA_GIRAR
                                self.stop_start_time = time.time()
                                self.command_queue.put({'v': 0.0, 'w': 0.0})
                            else:
                                # Mantiene velocidad lineal constante y velocidad angular nula
                                self.command_queue.put({'v': self.v, 'w': 0.0})
                                
                        elif self.fsm == BillarState.PARANDO_PARA_GIRAR:
                            # Esperar a que el robot se detenga físicamente antes de ordenar la rotación sobre su eje
                            if (time.time() - self.stop_start_time) >= self.stop_duration:
                                self.fsm = BillarState.GIRANDO
                                
                        elif self.fsm == BillarState.GIRANDO:
                            logger.info("Enviando comando para rotar 180 grados.")
                            # El firmware del Arduino ya cuenta con la rutina de giro preciso en grados
                            self.command_queue.put({'ang': 180.0})
                            self.giro_terminado = False
                            self.fsm = BillarState.ESPERANDO_GIRO
                            
                        elif self.fsm == BillarState.ESPERANDO_GIRO:
                            if self.giro_terminado:
                                logger.info("Giro completado con éxito. Reanudando avance.")
                                self.fsm = BillarState.AVANZA
                logger.debug(f"FSM: {self.fsm}")
                self.last_time=ahora
        
    
    def check_position_estimate(self):
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
            
        return x, y, theta
            


    def send_move(self, v, w):
        bouncer_agent.send(f"agent/{self.robot_id}/move", {'v': v, 'w': w})

# a partir de aqui es todo de recibir
#cuando conecta
def on_connect(client,userdata,flags,rc):
    pass
   
#cuando llega el mensaje
def on_message(client,userdata, msg):
     pass

# Configuración del Agente
if __name__ == "__main__":
   # Configuración del Agente
    bouncer_agent = Agent(
      device_class=BouncerRobot,
      id='Bouncer Robot',
      ip='192.168.10.1',
      data_port = 5563,
      hub_ip='192.168.10.1'
    )
    # 1. Creamos un manejador de consola (StreamHandler)
    logger = setup_logger(bouncer_agent.device.log_file,console_level=logging.WARNING)
    logger.propagate=False # Evita que los mensajes se dupliquen si el logger raíz también tiene handlers
    time.sleep(1)    
    
    t = threading.Thread(target=bouncer_agent.device.run)
    t.daemon = True # Se cierra cuando cierres el programa principal
    t.start()
    
    # El agente se queda escuchando MQTT
    #bouncer_agent.listen()
    #MQTT_agent.register()
    logger.info(f'Agent {bouncer_agent.id} is listening')

    topic=b'6/pos'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=b'arena/boundaries'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=b'agent/6/feedback'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=b'agent/6/wheel'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    def mqtt_and_dispatch():
        # Configurar MQTT aquí...
       
        
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER, PUERTO, 60)
        client.loop_start()
        logger.info(f"Agent {bouncer_agent.id} en marcha")
        logger.info(f"Agente {bouncer_agent.id} y despachador en marcha.")
       
        # 3. Hilo Principal: Despachador de la cola hacia ZeroMQ (ZMQ)
        while True:
            if not bouncer_agent.device.command_queue.empty():
                cmd = bouncer_agent.device.command_queue.get()
                
                # Determinamos el tópico ZMQ adecuado según el tipo de comando
                if 'ang' in cmd :
                    # Si es una operación compleja de giro, la mandamos al topic turn para giro preciso con eng en grados
                    topic = f"agent/{bouncer_agent.device.robot_id}/turn"
                    logger.debug(f"Enviado {cmd}")
                else:
                    # Si es velocidad cruda (v, w), va al tópico tradicional de movimiento
                    topic = f"agent/{bouncer_agent.device.robot_id}/move"
                     
                try:
                    bouncer_agent.send(topic, cmd)
                    logger.debug(f"Despachado a ZMQ -> {topic}: {cmd}")
                except Exception as e:
                    logger.error(f"Error enviando por ZMQ: {e}")
                    
                bouncer_agent.device.command_queue.task_done()
            time.sleep(0.001)
    
    mqtt_and_dispatch()
    # Configuración MQTT
    

    
