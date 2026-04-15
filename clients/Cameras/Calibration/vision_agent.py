# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  vision_agent.py
# FECHA:    15 de Abril, 2026
# OBJETIVO: Agente que procesa la unión de cámaras y envía el frame resultante
#           al RobotariumHub para su distribución a otros agentes.
# ==================================================================================

import cv2
import numpy as np
import zmq
import base64
import json
import time

class VisionAgent:
    def __init__(self, hub_ip="192.168.10.1", hub_data_port=5556, h_file="homography_matrix.npy"):
        # 1. Conexión con el Hub (Como publicador hacia el Hub)
        # Nota: En tu arquitectura, el Hub hace bind, el agente hace connect
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect(f"tcp://{hub_ip}:{hub_data_port}")
        
        # 2. Configuración de cámaras y calibración
        self.H = np.load(h_file)
        self.cap_a = cv2.VideoCapture(5) # Índice según tu script de comprobación
        self.cap_b = cv2.VideoCapture(1)
        
        self._init_stitching_params()

    def _init_stitching_params(self):
        # Tomamos frames para calcular dimensiones finales
        ret_a, frame_a = self.cap_a.read()
        ret_b, frame_b = self.cap_b.read()
        
        h_a, w_a = frame_a.shape[:2]
        h_b, w_b = frame_b.shape[:2]

        esquinas_b = np.array([[0,0],[w_b,0],[w_b,h_b],[0,h_b]], dtype=np.float32).reshape(-1,1,2)
        esquinas_b_trans = cv2.perspectiveTransform(esquinas_b, self.H)

        x_min = min(0, np.min(esquinas_b_trans[:,:,0]))
        y_min = min(0, np.min(esquinas_b_trans[:,:,1]))
        x_max = max(w_a, np.max(esquinas_b_trans[:,:,0]))
        y_max = max(h_a, np.max(esquinas_b_trans[:,:,1]))

        self.offset_x, self.offset_y = -int(x_min), -int(y_min)
        self.total_w, self.total_h = int(x_max - x_min), int(y_max - y_min)

        self.H_offset = self.H.copy()
        self.H_offset[0,2] += self.offset_x
        self.H_offset[1,2] += self.offset_y

    def run(self):
        print("Agente de Visión conectado al Hub...")
        try:
            while True:
                ret_a, frame_a = self.cap_a.read()
                ret_b, frame_b = self.cap_b.read()

                if ret_a and ret_b:
                    # Stitching
                    canvas = cv2.warpPerspective(frame_b, self.H_offset, (self.total_w, self.total_h))
                    canvas[self.offset_y:self.offset_y+frame_a.shape[0], 
                           self.offset_x:self.offset_x+frame_a.shape[1]] = frame_a

                    # Codificación para envío ligero
                    _, buffer = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')

                    # Publicar al Hub siguiendo el esquema del Robotarium
                    # Tópico: vision/stitched
                    topic = "vision/stitched"
                    payload = {
                        "source_id": "vision_agent_01",
                        "topic": topic,
                        "payload": {
                            "image": jpg_as_text,
                            "width": self.total_w,
                            "height": self.total_h
                        }
                    }
                    
                    # Envío siguiendo la lógica de multipart de tu hub.py
                    self.socket.send_string(topic, flags=zmq.SNDMORE)
                    self.socket.send_json(payload)

                time.sleep(0.04) # ~25 FPS

        except KeyboardInterrupt:
            print("Cerrando agente...")
        finally:
            self.cap_a.release()
            self.cap_b.release()

if __name__ == "__main__":
    agent = VisionAgent()
    agent.run()