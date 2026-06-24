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
import argparse
import logging

BROKER = "192.168.10.1"
PUERTO = 1883



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
        self.safety_distance = 40.0 
        self.t_retrocediendo=0
        self.control_time=0.05 #ms
        self.last_time=0
        self.command_queue = Queue() # Cola para enviar comandos al agente
        # Cola de mensajes recibidos
        incoming_queue = Queue(maxsize=500) # Cola generosa para absorber ráfagas
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
        self.Kp_gira = 2.5            # Ganancia Proporcional para controlar la velocidad de giro
        self.tolerance_theta = 0.05    # Tolerancia de error angular en radianes (~2.8 grados)
        # Guardará datos con el formato: { id_robot: {'x': x, 'y': y, 'theta': theta, 'last_update': timestamp} }
        self.posiciones_enjambre = {}
        self.radio_enjambre=1.0
   
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

    
    '''
    def on_data(self, topic: str, message: str) -> None:
        logger.debug(f"Incoming data. Topic {topic}, mensaje {message}")
        #Handle incoming data
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
        else:
            
                # Extraemos el ID del robot desde el propio tópico (ej: "5/pos" -> 5)
                parts = topic.split('/')
                logger.critical(parts)
                logger.critical(parts[0])
                id_remoto = int(parts[0])
                try:
                    # 2. Limpieza del string de datos
                    # A veces ZMQ mete los JSON en formato "['{...}']" o añade caracteres de escape.
                    clean_message = message.strip()
                    if clean_message.startswith("['") and clean_message.endswith("']"):
                        clean_message = clean_message[2:-2]
                    elif clean_message.startswith("[") and clean_message.endswith("]"):
                        clean_message = clean_message[1:-1]
            
                    clean_message = clean_message.strip().strip("'").strip('"')
                    raw_data= json.loads(clean_message)
                
                    if isinstance(raw_data, str):
                        raw_data = json.loads(raw_data)
                    x = float(raw_data.get('x'))
                    y = float( raw_data.get('y'))
                    theta = float (raw_data.get('yaw'))
                    sent_time = raw_data.get("timestamp")
                    # Enviamos los datos a la función del robot
                    bouncer_agent.device.actualizar_con_vecinos(id_remoto, x, y, theta,sent_time)
                except Exception as e:
                    logger.error(f"Error al decodificar topic: {topic} {e}")
    '''
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
        d_left = abs(x - x_min)
        d_right = abs(x_max - x)
        d_bottom = abs(y - y_min)
        d_top = abs(y_max - y)
        #logger.info(f"Limites: x {x_min} ,{x_max}, y {y_min}, {y_max}")
        logger.info(f"Distancias a paredes: Left: {d_left:.2f}, Right: {d_right:.2f}, Top: {d_top:.2f}, Bottom: {d_bottom:.2f}")       
                    
        return [d_left,d_right,d_top,d_bottom]
    
    def calcular_distancia_en_movimiento(self, x, y, theta):
        """
        Calcula la distancia hasta la pared que intersecta la trayectoria frontal del robot,
        contemplando que yaw=0 es mirando hacia ARRIBA (eje +Y).
        """
        # Obtenemos las componentes de movimiento según el convenio de tu odometría
        # Si dx = v * cos(theta) y dy = v * sin(theta), entonces:
        # cos_t representa el avance en X, sin_t representa el avance en Y.
        # Obtenemos las direcciones de movimiento según dictamina tu odometría
        dir_x = math.cos(theta) 
        dir_y = math.sin(theta)
        
        x_min, x_max = self.boundaries[0], self.boundaries[1]
        y_min, y_max = self.boundaries[2], self.boundaries[3]
        
        candidatos = {}
        
       
        # 0. Casos evidentes
        if abs(theta)<0.1: 
            candidatos['arriba']=abs(y_max-y)
        elif abs(theta-np.pi/2.0)<0.1:
            candidatos['derecha']=abs(x_max-x)
        elif abs(theta-np.pi)<0.1:
            candidatos['abajo']=abs(y-y_min)
        elif abs(theta-3*np.pi/2.0)<0.1:
            candidatos['izquierda']=abs(x_min-x)
        else:
            # 1. Intersección con componentes de avance en X (Paredes Izquierda y Derecha)
            if theta < 0 and theta > -np.pi/2.0: # El modelo matemático dice que se mueve hacia la Derecha (+X)
                candidatos['derecha'] = (x_max - x) /dir_y
                candidatos['arriba']=(y_max-y) / dir_x
            elif theta <= -np.pi/2.0 and theta > -np.pi: # El modelo matemático dice que se mueve hacia la Izquierda (-X)
                candidatos['derecha'] = abs((y_min - y) /dir_x)
                candidatos['abajo'] = abs((x_max - x) / dir_y)
            elif theta >np.pi/2 and theta < np.pi:
                candidatos['izquierda'] = abs((x - x_min) /dir_y)
                candidatos['abajo'] = abs((y_min - y) / dir_y)
            else:
                candidatos['izquierda'] = abs((x - x_min) / dir_y)
                candidatos['arriba'] = abs((y_max -y) / dir_x)
                
        logger.critical(f"Theta: {theta}, Candidatos: {candidatos}")
            # Filtramos para quedarnos SOLO con distancias reales hacia adelante (positivas)
        candidatos_validos = {k: v for k, v in candidatos.items() if v > 0}

        if not candidatos_validos:
            if abs(x_max-x)<1e-5:
                candidatos['derecha']=0
            elif abs(x-x_min)<1e-5:
                candidatos['izquierda']=0
            if abs(y_max-y)<1e-5:
                candidatos['arriba']=0
            elif abs(y-y_min)<1e-5:
                candidatos['abajo']=0

        # La pared de impacto real será la que esté más cerca en la trayectoria
        if not candidatos_validos:
            pared_impacto=None
            distancia_proyectada=100
        else:
            pared_impacto = min(candidatos_validos, key=candidatos_validos.get)
            distancia_proyectada = candidatos_validos[pared_impacto]
        
        return pared_impacto, distancia_proyectada
    
    def calcular_reflexion(self, pared, theta_actual):
        """
        Calcula el ángulo de reflexión perfecta bajo el convenio:
        yaw = 0 mirando hacia arriba (+Y), crece antihorario.
        """
        theta_deg = math.degrees(theta_actual) % 360
        nuevo_theta=0

        if pared in ['arriba', 'abajo']:
            # Se refleja respecto al eje horizontal (invierte componente Y)
            # En tu convenio '0' es arriba, por lo tanto la reflexión horizontal es (180 - theta)
            nuevo_theta = (180 - theta_deg) % 360
        elif pared in ['izquierda', 'derecha']:
            # Se refleja respecto al eje vertical (invierte componente X)
            # En tu convenio, esto equivale a cambiar el signo de la desviación respecto a '0' (-theta)
            nuevo_theta = (-theta_deg) % 360

        if abs(abs(theta_deg - nuevo_theta) - 180) < 1.0:
            nuevo_theta = (nuevo_theta + 5) % 360    
        
        return math.radians(nuevo_theta)
    
    def actualizar_fsm(self,x,y,theta):
        # 1. Obtener métricas
        distancias= self.get_distance_to_wall(x, y,theta)
        dist_absoluta=np.min(distancias)
        
        pared_abs=min(range(len(distancias)), key=lambda i: distancias[i])
        pared_mov, dist_movimiento = self.calcular_distancia_en_movimiento(x, y, theta)
        logger.warning(f"Distancia absoluta: {dist_absoluta}, distancia proyectada: {dist_movimiento}, pared: {pared_mov}")
        # 2. Evaluación de la Máquina de Estados
        if self.fsm == RobotState.AVANZA:
            # FILTRO 1: Seguridad Absoluta. Si por colisión o inercia está pegado a un muro, va atrás.
            if np.abs(dist_absoluta) <= self.danger_distance: 
                self.fsm= RobotState.PARANDO_PARA_RETROCEDER
                self.t_retrocediendo=time.time()
                # Se guarda la pared absoluta de la que se debe alejar
                pared_de_escape = pared_abs 
                self.last_wall_hit=pared_de_escape

            # FILTRO 2: Comportamiento Billar. Si la trayectoria colisionará pronto, calcula reflexión.
            elif dist_movimiento <= self.safety_distance: 
                self.target_theta = self.calcular_reflexion(pared_mov, theta)
                self.fsm = RobotState.GIRA
                self.giro_terminado=False
        elif self.fsm == RobotState.PARANDO_PARA_RETROCEDER:
            if (time.time() - self.t_retrocediendo) >= self.stop_duration:
                self.fsm = RobotState.RETROCEDE
                self.t_retrocediendo=time.time()
        elif self.fsm == RobotState.RETROCEDE:
            if np.abs(dist_absoluta) >= self.safety_distance or (time.time()-self.t_retrocediendo)>self.retrocede_duration:
                self.target_angle = self.calcular_reflexion(pared_mov, theta)
                self.fsm = RobotState.GIRA
        elif self.fsm == RobotState.GIRA:   
            # Cambiamos la condición: ya no dependemos del feedback del Arduino
            # Calculamos el error angular instantáneo en radianes
            error_theta = self.target_theta - theta
            # Normalizamos el error en el rango [-pi, pi] para que siempre gire por el camino más corto
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            
            if abs(error_theta) <= self.tolerance_theta:
                logger.info("¡Giro continuo terminado por software!")
                self.giro_terminado = True
                self.fsm = RobotState.AVANZA

        # 6. Decisión de velocidad basada en FSM
        if self.fsm == RobotState.AVANZA:
            v = self.speed
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
        
        elif self.fsm == RobotState.PARANDO_PARA_RETROCEDER:
            v = 0.0
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
        
        elif self.fsm == RobotState.RETROCEDE:
            v = -25.0 # Reducido un poco para control suave en cm/s
            w = 0.0
            self.command_queue.put({'v': v, 'w': w})
            
        elif self.fsm == RobotState.GIRA:
            # Volvemos a calcular el error para aplicar el control proporcional
            error_theta = math.atan2(math.sin(self.target_theta - theta), math.cos(self.target_theta - theta))
        
            # El robot gira sobre su propio eje: v = 0, w proporcional al error
            v = 0.0 
            w = self.Kp_gira * error_theta
            # Limitamos la velocidad angular mínima para superar la zona muerta
            if w<1.0 and w>0.1:
                w=1.0
            elif w>-1 and w<-0.1:
                w=-1.0
        
            # Limitamos la velocidad angular máxima por seguridad física de los motores
            w_max = 3.0 # rad/s
            w = max(min(w, w_max), -w_max)
            

        
            self.command_queue.put({'v': v, 'w': w})

        self.fsm_last = self.fsm
        logger.warning(f"Estado FSM: {self.fsm.name} | Target Theta: {math.degrees(self.target_theta):.2f}° | Theta Act: {math.degrees(theta):.2f}°")
        logger.warning(f"v: {v} | w: {w}")
    
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

    def actualizar_con_vecinos(self, id_remoto, x, y, theta, sent_time):
        """
        Registra la posición recibida de cualquier robot del laboratorio y 
        muestra por pantalla lo que ve este robot actual.
        """
        # Guardamos o actualizamos la posición del robot que acaba de publicar
        self.posiciones_enjambre[id_remoto] = {
            'x': x,
            'y': y,
            'theta': theta,
            'sent_time': sent_time,
            'last_update': time.time()
        }
        
        # Imprimimos periódicamente (o cada vez que cambia) para verificar qué ve este robot
        # Para no saturar la consola, contamos cuántos vecinos tenemos guardados
        num_vecinos = len(self.posiciones_enjambre)
        
        logger.warning(f"\n--- [Robot {self.robot_id}] Estado del Enjambre ({num_vecinos} detectados) ---")
        for rid, datos in sorted(self.posiciones_enjambre.items()):
            # Marcamos con un asterisco si los datos pertenecen a uno mismo
            es_propio = " (YO)" if rid == self.robot_id else ""
            logger.warning(f"  > Robot {rid:02d}{es_propio}: X={datos['x']:.2f}, Y={datos['y']:.2f}, Theta={math.degrees(datos['theta']):.1f}°")
        logger.warning("-" * 50)

# a partir de aqui es todo de recibir
#cuando conecta
def on_connect(client,userdata,flags,rc):
    pass
   
#cuando llega el mensaje
def on_message(client,userdata, msg):
     try:
         # Meto el mensaje en la cola de mensajes entrantes para procesarlos en orden
         if not bouncer_agent.device.command_queue.full():
             bouncer_agent.device.command_queue.put_nowait((msg.topic, msg.payload.decode()))
     except Exception as e:
        logger.error(f"Error al procesar mensaje: {e}")

# Configuración del Agente
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lanzador de agente Robotarium con parámetros dinámicos.")
    parser.add_argument(
        "-i", "--id", 
        type=int, 
        required=True, 
        help="ID numérico del robot (ej. 5 o 6)"
    )
    parser.add_argument(
        "-p", "--port", 
        type=int, 
        required=True, 
        help="Puerto de datos (data_port) para el agente (ej. 5556)"
    )

    args = parser.parse_args()

    # 2. Configurar el logger usando el ID del robot dinámico
    agent_name = f"Robot_{args.id:02d}"
    logger = setup_logger(agent_name, console_level=logging.INFO)
    logger.info(f"Iniciando {agent_name} en el puerto de datos {args.port}...")

    # 3. Inicializar el agente ZMQ pasándole el puerto dinámico
   # Configuración del Agente
    bouncer_agent = Agent(
      device_class=BouncerRobot,
      id='Bouncer Robot',
      ip='192.168.10.1',
      data_port = args.port,
      hub_ip='192.168.10.1'
    )
    # 4. Asignar el ID correcto dentro del objeto hardware (BouncerRobot)
    bouncer_agent.device.robot_id = args.id

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

    topic=f'{args.id}/pos'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=b'arena/boundaries'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=f'agent/{args.id}/feedback'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    topic=f'agent/{args.id}/wheel'
    bouncer_agent.setup_subscriptions(topic)
    logger.info(f"Suscrito a topic {topic}")
    
    #Me tengo que suscribir a los topics de pos de todos los robots para poder calcular la distancia a los demás
    for robot_id in range(1, 10):  # Asumiendo que hay 10 robots en total
        if robot_id != args.id:  # No nos suscribimos a nuestro propio topic
            topic = f'{robot_id}/pos'
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
            ''' Proceso los mensajes de entrada'''
            while not bouncer_agent.device.incoming_queue.empty():
                try:
                    topic, payload_bytes = bouncer_agent.device.incoming_queue.get_nowait()
                    payload_str = payload_bytes.decode('utf-8')
                    # TODO: Filtro de seguridad
                    data =json.loads(payload_str)
                    parts = topic.split('/')
                    if len(parts) >= 2 and parts[0].isdigit():
                        id_remoto = int(parts[0])
                        if id_remoto == bouncer_agent.device.robot_id:
                            bouncer_agent.device.status = "INICIALIZADO"
                            bouncer_agent.device.on_pos_received(data.get('x'), data.get('y'), data.get('yaw'))
                            current_time = time.time()
                            sent_time = data.get("timestamp")
   
                            latency = (current_time - sent_time) * 1000 # Latencia en ms
    
                            # Calcular frecuencia (Delta tiempo entre este mensaje y el anterior)
                            if hasattr(bouncer_agent.device, 'last_pos_time'):
                                freq = 1.0 / (current_time - bouncer_agent.device.last_pos_time)
                                logger.debug(f"Frecuencia: {freq:.2f} Hz | Latencia Red/Proc: {latency:.2f} ms")
                
                            bouncer_agent.device.last_pos_time = current_time
                            
                        else:
                            bouncer_agent.device.actualizar_con_vecinos(id_remoto, data.get('x'), data.get('y'), data.get('yaw'), data.get('timestamp'))
                    elif topic == "arena/boundaries":
                        bouncer_agent.device.boundaries = data.get("points", bouncer_agent.device.boundaries)
                    elif topic == f"agent/{bouncer_agent.device.robot_id}/wheel":
                        wl = float(data.get('Wleft'))
                        wr = float(data.get('Wright'))
                        bouncer_agent.device.on_odom_received(wl, wr)
                    elif topic == f"agent/{bouncer_agent.device.robot_id}/feedback":
                        status = data.get("status")
                        op = data.get("op")
                        if status == "done" and op == "turn":
                            logger.info("¡Confirmación recibida desde Arduino! Giro terminado exitosamente.")
                            bouncer_agent.device.giro_terminado = True
                    else:
                        logger.warning(f"Mensaje recibido en topic desconocido: {topic}")
                except Exception as e:
                    logger.error(f"Error procesando mensaje entrante: {e}") 
                finally:
                    bouncer_agent.device.incoming_queue.task_done()

            if not bouncer_agent.device.command_queue.empty():
                cmd = bouncer_agent.device.command_queue.get()
                
                # Determinamos el tópico ZMQ adecuado según el tipo de comando
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
    

    
