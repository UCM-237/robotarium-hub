# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  vision_agent.py
# FECHA:    15 de Abril, 2026
# OBJETIVO: Agente que procesa la unión de cámaras y envía el frame resultante
#           al RobotariumHub para su distribución a otros agentes.
# ==================================================================================
import cv2
import numpy as np
import base64
import time
from agent import Agent, Device

class VisionDevice: # Esta clase cumple el protocolo Device de tu agent.py
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        # Configuración de cámaras (como tenías en tu vision_agent.py)
        self.cap_a = cv2.VideoCapture(5)
        self.cap_b = cv2.VideoCapture(1)
        self.H = np.load("homography_matrix.npy")
        
        # Parámetros de stitching
        self.total_w = 1280 # Ajusta según tus cámaras
        self.total_h = 720
        self.offset_x = 0
        self.offset_y = 0

    def connect(self) -> None:
        print("[INFO] Sistema de visión listo y conectado al dispositivo.")

    def on_data(self, topic: str, message: str) -> None:
        # Aquí recibirías datos del Hub (ej. si el Hub te pide cambiar parámetros)
        print(f"[RECV] Dato recibido en tópico {topic}")

    def run(self):
        """Bucle principal de captura y envío"""
        try:
            while True:
                ret_a, frame_a = self.cap_a.read()
                ret_b, frame_b = self.cap_b.read()

                if ret_a and ret_b:
                    # Lógica de Stitching (simplificada aquí, usa la tuya)
                    # ... (Tu código de warpPerspective y unión) ...
                    canvas = frame_a # Supongamos que este es el resultado unido
                    
                    # Codificación
                    _, buffer = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')

                    # Usar el método 'send' de tu clase Agent
                    payload = {
                        "image": jpg_as_text,
                        "width": self.total_w,
                        "height": self.total_h
                    }
                    self.agent.send("vision/stitched", payload)

                time.sleep(0.04) # ~25 FPS
        except KeyboardInterrupt:
            self.cap_a.release()
            self.cap_b.release()

# --- INSTANCIACIÓN ---
if __name__ == "__main__":
    # 1. Instanciamos el Agente (esto hace el registro 'hello' automáticamente)
    # Cambia la IP por la de tu Hub
    mi_agente = Agent(
        device_class=VisionDevice, 
        id="VisionSystem01", 
        ip="127.0.0.1",      # Tu IP local
        hub_ip="192.168.10.1" # IP del Hub
    )

    # 2. El Agente ya creó el VisionDevice internamente, lo recuperamos y lanzamos
    vision_system = mi_agente.device
    vision_system.connect()
    vision_system.run()