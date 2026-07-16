# -*- coding: UTF-8 -*-
#!/bin/python3
import json
import time
import math
import logging
from agent import Agent
from logger_config import setup_logger
from datetime import datetime
from queue import Queue # Para comunicar hilos de forma segura
import paho.mqtt.client as mqtt

BROKER = "192.168.10.1"
PUERTO = 1883
# Límites del tatami esperados (puedes ajustarlos o recibirlos dinámicamente)
LIMITS = {
    "x_min": -150.0,
    "x_max": 1200.0,
    "y_min": -50.0,
    "y_max": 350.0
}

# Parámetros físicos máximos para validación de saltos bruscos
MAX_PHYSICAL_SPEED = 100.0  # cm/s (Velocidad máxima que puede alcanzar el robot)
MAX_LATENCY_MS = 250.0      # Latencia máxima aceptable en milisegundos

class PositionVerifierDevice:
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        self.last_positions = {}  # Guardará {robot_id: {"x", "y", "yaw", "timestamp"}}
        # --- Configuración del Logger ---
        self.log_file = f"verifier_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.incoming_queue = Queue(maxsize=500) # Cola generosa para absorber ráfagas
        
    def connect(self) -> None:
        #logger.info("Dispositivo de Verificación inicializado y listo.")
        pass

    def on_data(self,topic:str,message:str)->None:
        logger.info(f"Recibido mensaje {message} en el topic {topic}")
        try:
            if not message or '{' not in message:
                logger.debug(f"Mensaje {message} descartado en topic {topic}")
                return
            payload_str=message[message.find('{'):]

            # Meto el mensaje en la cola de mensajes entrantes para procesarlos en orden
            if not self.incoming_queue.full():
                self.incoming_queue.put_nowait((topic, payload_str))
        except Exception as e:
            logger.error(f"Error al procesar mensaje: {e}")

    def verify_position(self, robot_id: str, data: dict):
        x = data.get("x")
        y = data.get("y")
        yaw = data.get("yaw")
        t_stamp = data.get("timestamp", time.time())
        
        current_time = time.time()
        
        # 1. Validación de Límites (Out of Bounds)[cite: 6]
        if not (x >=LIMITS["x_min"] and x <=LIMITS["x_max"]) or not (y >=LIMITS["y_min"] and y <=LIMITS["y_max"]):
            logger.error(
                f"🚨 [R-{robot_id}] ¡FUERA DE LÍMITES! Posición recibida: ({x:.2f}, {y:.2f}). "
                f"Rangos aceptados: X[{LIMITS['x_min']},{LIMITS['x_max']}], Y[{LIMITS['y_min']},{LIMITS['y_max']}]"
            )
            return

        # 2. Validación de Orientación (Yaw)
        if not (-math.pi <= yaw <= math.pi):
            logger.warning(f"⚠️ [R-{robot_id}] Orientación (yaw) fuera de rango normal [-pi, pi]: {yaw:.3f} rad")[cite: 2]

        # 3. Validación de Latencia[cite: 6]
        latency = (current_time - t_stamp) * 1000.0
        if latency > MAX_LATENCY_MS:
            logger.warning(f"🐢 [R-{robot_id}] Latencia alta en la transmisión/captura: {latency:.1f} ms")
        # 4. Validación de Saltos Físicos / Teletransportaciones (Coherencia temporal)
        if robot_id in self.last_positions:
            prev = self.last_positions[robot_id]
            dt = current_time - prev["time_local"]
            
            if dt > 0:
                # Distancia euclidiana entre la posición anterior y la actual[cite: 6]
                distance = math.sqrt((x - prev["x"])**2 + (y - prev["y"])**2)
                speed = distance / dt  # cm/s aprox.
                
                # Frecuencia estimada de refresco
                freq = 1.0 / dt
                
                # Si la velocidad calculada excede el límite físico, hay un problema de tracking
                if speed > MAX_PHYSICAL_SPEED:
                    logger.critical(
                        f"❌ [R-{robot_id}] ¡SALTO ANÓMALO DETECTADO! Se ha 'movido' a {speed:.1f} cm/s "
                        f"(Distancia: {distance:.1f} cm en {dt*1000:.1f} ms). Posible falso positivo de ArUco."
                    )
                else:
                    logger.info(
                        f"✅ [R-{robot_id}] Ok | Pos: ({x:.1f}, {y:.1f}) | Yaw: {math.degrees(yaw):.1f}° | "
                        f"Freq: {freq:.1f} Hz | Latencia: {latency:.1f} ms"
                    )

        # Actualizar el histórico para el próximo ciclo
        self.last_positions[robot_id] = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "time_local": current_time
        }

#cuando conecta
def on_connect(client,userdata,flags,rc):
    pass
   
#cuando llega el mensaje
def on_message(client,userdata, msg):
     pass

if __name__ == "__main__":
    # Instanciar el agente de verificación[cite: 6]
    verifier_agent = Agent(
        device_class=PositionVerifierDevice,
        id='PositionVerifier',
        ip='192.168.10.1',      # IP de la Raspberry/PC donde corras el verificado[cite: 6]
        data_port=5570,         # Un puerto libre para este agente
        hub_ip='192.168.10.1'   # IP del Robotarium Hub[cite: 2, 5]
    )

    # Configurar Logger de Consola a nivel INFO para ver los reportes[cite: 3, 6]
    logger = setup_logger("VerifierAgent", console_level=logging.INFO)
    logger.propagate = False
    time.sleep(1)
    # Nos suscribimos a la posición de todos los robots posibles (0 a 9)
    # Si quieres comprobar un robot específico, por ejemplo el 6, suscríbete a b'6/pos'[cite: 6]
    for r_id in range(10):
        topic = f"{r_id}/pos".encode('utf-8')
        verifier_agent.setup_subscriptions(topic)
        logger.info(f"Suscrito a la posición del robot {r_id} en el tópico: {topic}")
    def mqtt_and_dispatch():
        # Configurar MQTT aquí...
       
        
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER, PUERTO, 60)
        client.loop_start()
        
       
        # 3. Hilo Principal: Despachador de la cola hacia ZeroMQ (ZMQ)
        while True:
            ''' Proceso los mensajes de entrada'''
            while not verifier_agent.device.incoming_queue.empty():
                try:
                    topic, payload_str= verifier_agent.device.incoming_queue.get_nowait()
                    if '{' in payload_str:
                        # Cortamos todo lo que haya antes de la primera llave '{' por si hay restos del topic
                        payload_str = payload_str[payload_str.find('{'):]
                    else:
                        # Si ni siquiera tiene una llave de apertura, es basura pura de red. Descartamos.
                        continue
                    #payload_str = payload_bytes.decode('utf-8')
                    # TODO: Filtro de seguridad
                    data =json.loads(payload_str)
                    parts = topic.split('/')
                    logger.debug(f"procesando {parts}")
                    if len(parts) >= 2 and parts[0].isdigit():
                        id_remoto = int(parts[0])
                        verifier_agent.device.status = "INICIALIZADO"
                        verifier_agent.device.verify_position(id_remoto,data)
                            
                    else:
                        logger.warning(f"Mensaje recibido en topic desconocido: {topic}")
                except Exception as e:
                    logger.error(f"Error procesando mensaje entrante: {e}") 
                finally:
                    verifier_agent.device.incoming_queue.task_done()
    # Iniciamos la escucha de datos
    try:
        logger.info("Agente Verificador en marcha. Analizando flujos de posición...")
        verifier_agent.listen()
    except KeyboardInterrupt:
        logger.info("Apagando agente verificador...")