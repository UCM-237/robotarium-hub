# -*- coding: UTF-8 -*-
#!/bin/python3
import json
import time
import math
import logging
from agent import Agent
from logger_config import setup_logger

# Límites del tatami esperados (puedes ajustarlos o recibirlos dinámicamente)
LIMITS = {
    "x_min": -150.0,
    "x_max": 350.0,
    "y_min": -50.0,
    "y_max": 250.0
}

# Parámetros físicos máximos para validación de saltos bruscos
MAX_PHYSICAL_SPEED = 100.0  # cm/s (Velocidad máxima que puede alcanzar el robot)
MAX_LATENCY_MS = 250.0      # Latencia máxima aceptable en milisegundos

class PositionVerifierDevice:
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        self.last_positions = {}  # Guardará {robot_id: {"x", "y", "yaw", "timestamp"}}
        
    def connect(self) -> None:
        logger.info("Dispositivo de Verificación inicializado y listo.")

    def on_data(self, topic: str, message: str) -> None:
        # Esperamos tópicos con formato: {robot_id}/pos (ej. "6/pos")[cite: 2, 6]
        if topic.endswith("/pos"):
            try:
                robot_id = topic.split('/')[0]
                data = json.loads(message)
                if isinstance(data, str):
                    data = json.loads(data)
                
                self.verify_position(robot_id, data)
            except Exception as e:
                logger.error(f"Error procesando mensaje en {topic}: {e}")

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

    # Nos suscribimos a la posición de todos los robots posibles (0 a 9)
    # Si quieres comprobar un robot específico, por ejemplo el 6, suscríbete a b'6/pos'[cite: 6]
    for r_id in range(10):
        topic = f"{r_id}/pos".encode('utf-8')
        verifier_agent.setup_subscriptions(topic)
        logger.info(f"Suscrito a la posición del robot {r_id} en el tópico: {topic}")

    # Iniciamos la escucha de datos
    try:
        logger.info("Agente Verificador en marcha. Analizando flujos de posición...")
        verifier_agent.listen()
    except KeyboardInterrupt:
        logger.info("Apagando agente verificador...")