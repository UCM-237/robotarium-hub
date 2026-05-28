# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  vision_agent.py
# FECHA:    15 de Abril, 2026
# OBJETIVO: Agente que procesa la unión de cámaras y envía el frame resultante
#           al RobotariumHub para su distribución a otros agentes.
# ==================================================================================
import argparse

import cv2
import numpy as np
import base64
import time
from agent import Agent, Device
from threading import Thread
from logger_config import setup_logger
import logging

MAX_WIDHT=1280
MAX_HEIGHT=720

class VisionDevice: # Esta clase cumple el protocolo Device de tu agent.py
    def __init__(self, agent: Agent) -> None:
        self.agent = agent
        self.running = False
        self.gui = True
        # Configuración de cámaras (como tenías en tu vision_agent.py)
        self.cap_a = cv2.VideoCapture(0)
        #self.cap_a.set(cv2.CAP_PROP_BRIGHTNESS,200)
        self.cap_a.set(cv2.CAP_PROP_FPS,10)
        self.cap_a.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.cap_a.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap_a.set(cv2.CAP_PROP_EXPOSURE, -7)
        self.cap_a.set(cv2.CAP_PROP_BRIGHTNESS, 100) # Un valor medio/bajo
        
        self.cap_b = cv2.VideoCapture(2)
        #self.cap_b.set(cv2.CAP_PROP_BRIGHTNESS,80)
        self.cap_b.set(cv2.CAP_PROP_FPS,10)
        self.cap_b.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.cap_b.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap_b.set(cv2.CAP_PROP_EXPOSURE, -7)
        # 3. Opcional: Bajar el brillo si la exposición no es suficiente
        self.cap_b.set(cv2.CAP_PROP_BRIGHTNESS, 100)
        self.H = np.load("homography_matrix.npy")
        
        
        # Parámetros de stitching
        self.total_w = 1280 # Ajusta según tus cámaras
        self.total_h = 720
        self.offset_x = 0
        self.offset_y = 0

    def connect(self) -> None:
        print("[INFO] Sistema de visión listo y conectado al dispositivo.")
        #mi_agente.register()
        self.main_thread = Thread(target=self.run, args=())
        self.main_thread.start()

    def on_data(self, topic: str, message: str) -> None:
        # Aquí recibirías datos del Hub (ej. si el Hub te pide cambiar parámetros)
        #print(f"[RECV] Dato recibido en tópico {topic}")
        logger.debug(f"Dato recibido en tópico {topic}")
        self.running=True

    def run(self):
        """Bucle principal de captura y envío"""
        try:
            self.running = True
            while self.running:
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
                    if self.gui:
                        cv2.imshow("Stitching", canvas_red)
                        cv2.waitKey(1)
                    # Codificación
                    _, buffer = cv2.imencode('.jpg', canvas_red, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    #Cambio a PNG va de 0 a 9 (siendo 0 sin compresión y más rápido)
                    #_, buffer = cv2.imencode('.png', canvas_red, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')

                    # Usar el método 'send' de tu clase Agent
                    payload = {
                        "image": jpg_as_text,
                        "width": self.total_w,
                        "height": self.total_h
                    }
                    self.agent.send("vision/stitched", payload)
                    logger.debug("Frame enviado al Hub")
                time.sleep(0.001) # ~25 FPS
        except KeyboardInterrupt:
            self.cap_a.release()
            self.cap_b.release()
            print("Se sale en la excepcion")

# --- INSTANCIACIÓN ---
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gui', action='store_true', help="Ejecutar sin ventana de video")
    args = parser.parse_args()

    # 1. Instanciamos el Agente (esto hace el registro 'hello' automáticamente)
    # Cambia la IP por la de tu Hub
    agente_vision = Agent(
        device_class=VisionDevice, 
        id="VisionSystem05", 
        ip="192.168.10.1",      # Tu IP local
        data_port=5559,
        hub_ip="192.168.10.1" # IP del Hub
    )

    agente_vision.device.gui = not args.no_gui # Seteamos el modo de visualización
    # 1. Creamos un manejador de consola (StreamHandler)
    fname=f"vision_agent_log_{time.strftime('%Y%m%d_%H%M%S')}"
    logger = setup_logger(fname,console_level=logging.WARNING) # Cambia a DEBUG para ver todos los mensajes
    logger.propagate = False # Evita que los mensajes se propaguen al logger raíz (y se dupliquen)
    time.sleep(1)    

  
