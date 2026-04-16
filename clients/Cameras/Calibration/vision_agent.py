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

MAX_WIDHT=1280
MAX_HEIGHT=720

class VisionDevice: # Esta clase cumple el protocolo Device de tu agent.py
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        # Configuración de cámaras (como tenías en tu vision_agent.py)
        self.cap_a = cv2.VideoCapture(5)
        self.cap_a.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.cap_b = cv2.VideoCapture(1)
        self.cap_b.set(cv2.CAP_PROP_BUFFERSIZE,1)
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
                if not ret_a and not ret_b:
                    continue
                else:
                    h_a, w_a, _ = frame_a.shape
                    h_b, w_b, _ = frame_b.shape
                    # --- CALCULAR OFFSET PARA EVITAR NEGROS ---
                    esquinas_b = np.array([[0,0],[w_b,0],[w_b,h_b],[0,h_b]], dtype=np.float32).reshape(-1,1,2)
                    esquinas_b_trans = cv2.perspectiveTransform(esquinas_b, self.H)

                    x_min = min(0, np.min(esquinas_b_trans[:,:,0]))
                    y_min = min(0, np.min(esquinas_b_trans[:,:,1]))
                    x_max = max(w_a, np.max(esquinas_b_trans[:,:,0]))
                    y_max = max(h_a, np.max(esquinas_b_trans[:,:,1]))

                    offset_x = -int(x_min)
                    offset_y = -int(y_min)
                    nuevo_w = int(x_max - x_min)
                    nuevo_h = int(y_max - y_min)

                    H_offset = self.H.copy()
                    H_offset[0,2] += offset_x
                    H_offset[1,2] += offset_y

                    # --- CREAR LIENZO Y UNIR ---
                    canvas = np.zeros((nuevo_h, nuevo_w, 3), dtype=np.uint8)
                    canvas[offset_y:offset_y+h_a, offset_x:offset_x+w_a] = frame_a

                    frame_b_warped = cv2.warpPerspective(frame_b, H_offset, (nuevo_w, nuevo_h))
                    mask = frame_b_warped > 0
                    canvas[mask] = frame_b_warped[mask]

                    # --- REDIMENSIONAR PARA VER EN PANTALLA ---
                    def rescale_frame(frame, max_w, max_h):
                        alto, ancho = frame.shape[:2]
                        escala = min(max_w / ancho, max_h / alto)
                        nuevo_ancho = int(ancho * escala)
                        nuevo_alto = int(alto * escala)
                        return cv2.resize(frame, (nuevo_ancho, nuevo_alto), interpolation=cv2.INTER_AREA)

                    canvas_red = rescale_frame(canvas, MAX_WIDHT, MAX_HEIGHT)

                    # --- MOSTRAR ---
                    cv2.imshow("Stitching Completo", canvas_red)
                    cv2.waitKey(1)
                    # Codificación
                    _, buffer = cv2.imencode('.jpg', canvas_red, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')

                    # Usar el método 'send' de tu clase Agent
                    payload = {
                        "image": jpg_as_text,
                        "width": self.total_w,
                        "height": self.total_h
                    }
                    self.agent.send("vision/stitched", payload)
                    print("Enviada imagen")

                time.sleep(0.04) # ~25 FPS
        except KeyboardInterrupt:
            self.cap_a.release()
            self.cap_b.release()
            print("Se sale en la excepcion")

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