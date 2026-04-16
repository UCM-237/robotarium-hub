import cv2
import numpy as np
import base64
import json
from agent import Agent, Device

class ArucoDevice:
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        self.window_name = "Robotarium - Recepcion Vision"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def connect(self) -> None:
        print(f"[INFO] Agente {self.agent.id} conectado y esperando video...")

    def on_data(self, topic: str, message: str) -> None:
        """
        Este método es llamado automáticamente por agent.py 
        cuando llega un mensaje al tópico suscrito.
        """
        if topic == "vision/stitched":
            try:
                # 1. Convertir el string JSON a diccionario
                data = json.loads(message)
                
                # 2. Extraer la imagen en Base64 y decodificarla
                # Basado en el payload que envía tu vision_agent.py
                img_b64 = data['payload']['image']
                img_bytes = base64.b64decode(img_b64)
                
                # 3. Convertir bytes a imagen de OpenCV
                np_array = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Aquí es donde más adelante meterás: detect_arucos(frame)
                    self.show_frame(frame)
                    
            except Exception as e:
                print(f"[ERROR] Error al procesar frame: {e}")

    def show_frame(self, frame):
        cv2.imshow(self.window_name, frame)
        # IMPORTANTE: waitKey es vital para que la ventana se refresque
        cv2.waitKey(1)

    def run(self):
        # Este agente es pasivo, solo reacciona a on_data
        # Mantenemos el hilo principal vivo
        while True:
            import time
            time.sleep(1)

# --- LANZAMIENTO DEL AGENTE ---
if __name__ == "__main__":
    # IMPORTANTE: En agent.py, asegúrate de añadir la suscripción 
    # al tópico 'vision/stitched' en el método listen()
    
    aruco_agent = Agent(
        device_class=ArucoDevice,
        id="ArucoTracker",
        ip="127.0.0.1",        # Tu IP
        hub_ip="192.168.10.1"  # IP del Hub
    )
    
    # Iniciamos el bucle pasivo
    aruco_agent.device.connect()
    aruco_agent.device.run()