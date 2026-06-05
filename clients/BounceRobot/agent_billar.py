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


class RobotState(Enum):
    AVANZA = 1
    GIRA = 2
    PARA = 3
    PARANDO_PARA_RETROCEDER = 4
    RETROCEDE = 5
    PARANDO_PARA_GIRAR = 6
    ESPERANDO_GIRO = 7
    
class BouncerRobot:
    def __init__(self, agent: Agent) -> None:
        '''The constructor optionally receive a list of listeners'''
        self.boundaries=[-102.0,298.0,16,160.0] #Lo inicializo asi por si acaso no recibe los limites
        self.margin = 20.0
        self.speed = 20.0
        self.fsm = RobotState.AVANZA
        self.fsm_last=RobotState.AVANZA
        self.danger_distance = 20.0
        self.last_wall_hit=None
        self.v=35.0
        self.w=3.0
        self.direction=[0.707, 0.707]
        self.robot_id=6
        self.pos=[0.0,0.0,0.0]
        self.angular_speed=1.0
        self.safety_distance = 30.0 
        self.t_retrocediendo=0
        self.control_time=0.05 #ms
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
            self.status = "INICIALIZADO"
            try:
                raw_data= json.loads(message)
                #print(raw_data)
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

    def run(self):
        """Bucle de control independiente que corre a ~20Hz"""
        while True:
            ahora = time.time()
            if (ahora-self.last_time) >= self.control_time:
                # 1. VERIFICACIÓN DE SEGURIDAD (WATCHDOG)
                # Si hace más de 1.5 segundos que no sabemos nada del robot...
                time_since_vision = ahora - self.last_pos_time
                time_since_odom = ahora - self.last_odom_time
                if self.status=="ESPERANDO POSICIÓN INICIAL":
                    logger.warning("Esperando posición inicial... Aún no se han recibido datos de visión.")
                    self.command_queue.put({'v': 0.0, 'w': 0.0})
                else:
                    if time_since_vision > 2.5 and time_since_odom > 2.5:
                        logger.warning("SISTEMA DESCONECTADO: Parando robot por seguridad")
                        self.command_queue.put({'v': 0.0, 'w': 0.0})
                    else:
                        # 2. EJECUCIÓN DE LA LÓGICA
                        # pos_logic ahora decidirá qué posición usar
                        x,y,theta=self.check_position_estimate()
                        logger.info(f"Usando posición {self.status}: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
                        self.actualizar_fsm(x,y,theta)
                self.last_time=ahora
            

  
        
    def get_distance_to_wall(self, x, y, theta):
        # 1. Límites actuales (centímetros)
        x_min, x_max = self.boundaries[0], self.boundaries[1]
        y_min, y_max = self.boundaries[2], self.boundaries[3]

        # 2. Distancias Euclidianas "puras" (¿A cuánto estoy de las bandas?)
        d_left = x - x_min
        d_right = x_max - x
        d_bottom = y - y_min
        d_top = y_max - y
        #logger.info(f"Limites: x {x_min} ,{x_max}, y {y_min}, {y_max}")
        logger.info(f"Distancias a paredes: Left: {d_left:.2f}, Right: {d_right:.2f}, Top: {d_top:.2f}, Bottom: {d_bottom:.2f}")       
                    
        return [d_left,d_right,d_top,d_bottom]
    
    def calcular_distancia_en_movimiento(self, x, y, theta):
        """
        Calcula la distancia hasta la pared que intersecta la trayectoria frontal del robot.
        Retorna la pared de impacto proyectada y la distancia en metros.
        """
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        x_min, x_max = self.boundaries[0], self.boundaries[1]
        y_min, y_max = self.boundaries[2], self.boundaries[3]
        candidatos = {}
        
        # 1. Intersección con paredes verticales (Eje X)
        if cos_t > 1e-6: # Moviéndose hacia la derecha (+X)
            dist_x = (x_max - x) / cos_t
            candidatos['derecha'] = dist_x
        elif cos_t < -1e-6: # Moviéndose hacia la izquierda (-X)
            dist_x = (x_min - x) / cos_t  # Será positivo porque (x_min - x) es negativo y cos_t es negativo
            candidatos['izquierda'] = dist_x
            
        # 2. Intersección con paredes horizontales (Eje Y)
        if sin_t > 1e-6: # Moviéndose hacia arriba (+Y)
            dist_y = (y_max - y) / sin_t
            candidatos['arriba'] = dist_y
        elif sin_t < -1e-6: # Moviéndose hacia abajo (-Y)
            dist_y = (y_min - y) / sin_t
            candidatos['abajo'] = dist_y

        # Si por algún motivo matemático la lista está vacía, evitamos errores
        if not candidatos:
            return None, float('inf')
            
        # La pared real de impacto será la que requiera recorrer la menor distancia positiva
        pared_impacto = np.min(list(candidatos.values()))
        distancia_proyectada = candidatos[pared_impacto]
        
        return pared_impacto, distancia_proyectada

    def calcular_reflexion(self, pared, theta_actual):
        """Calcula el ángulo de reflexión perfecta."""
        # Trabajamos en grados para facilitar la lectura del log, luego convierte si es necesario
        theta_deg = math.degrees(theta_actual) % 360
        nuevo_theta = theta_deg    
        if pared in ['izquierda', 'derecha']:
            nuevo_theta = (180 - theta_deg) % 360
        elif pared in ['arriba', 'abajo']:
            nuevo_theta = (-theta_deg) % 360
            
        return math.radians(nuevo_theta)
    
    def actualizar_fsm(self,x,y,theta):
        # 1. Obtener métricas
        pared_abs, dist_absoluta = self.get_distance_to_wall(x, y,theta)
        pared_mov, dist_movimiento = self.calcular_distancia_en_movimiento(x, y, theta)
        
        # 2. Evaluación de la Máquina de Estados
        if self.fsm == RobotState.AVANZA:
            # FILTRO 1: Seguridad Absoluta. Si por colisión o inercia está pegado a un muro, va atrás.
            if dist_absoluta <= self.danger_distance: 
                estado = RobotState.PARANDO_PARA_RETROCEDER
                self.t_retrocediendo=time.time()
                # Se guarda la pared absoluta de la que se debe alejar
                pared_de_escape = pared_abs 
                self.last_wall_hit=pared_de_escape

            # FILTRO 2: Comportamiento Billar. Si la trayectoria colisionará pronto, calcula reflexión.
            elif dist_movimiento <= self.safety_distance: 
                target_angle = self.calcular_reflexion(pared_mov, theta)
                estado = RobotState.GIRA
        elif self.fsm == RobotState.PARANDO_PARA_RETROCEDER:
            if (time.time() - self.t_retrocediendo) >= self.stop_duration:
                estado = RobotState.RETROCEDE
        elif self.fsm == RobotState.RETROCEDE:
            if dist_absoluta >= self.safety_distance:
                estado = RobotState.GIRA
        elif self.fsm == RobotState.GIRA:   
            if self.giro_terminado:
                estado = RobotState.AVANZA

        # 6. Decisión de velocidad basada en FSM
        if self.fsm== RobotState.AVANZA:
            v = self.speed
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
            
            
        elif self.fsm == RobotState.PARANDO_PARA_RETROCEDER and self.fsm_last!=RobotState.PARANDO_PARA_RETROCEDER:
            v = 0.0
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
            
        elif self.fsm == RobotState.RETROCEDE and self.fsm_last!=RobotState.RETROCEDE:
            v = -40 
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
            
        elif self.fsm == RobotState.PARANDO_PARA_GIRAR and self.fsm_last!=RobotState.PARANDO_PARA_GIRAR:
            v = 0.0
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
            
        elif self.fsm == RobotState.GIRA:
            #Pasamos self.target_theta a grados porque el Arduino lo espera así para la operación de giro preciso
            self.target_theta = math.degrees(self.target_theta)
            #TEST. Remove
            self.target_theta=-180
            comando_giro = {'op': 'turn', 'ang': self.target_theta}
            self.command_queue.put({'ang': self.target_theta})
        self.fsm_last=self.fsm
        logger.warning(f"Estado FSM: {self.fsm.name} | Target Wall: {self.last_wall_hit} | Target Theta: {self.target_theta:.2f}° |theta: {math.degrees(theta)} ") 

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
    

    
