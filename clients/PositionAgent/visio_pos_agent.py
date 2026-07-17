import argparse
import cv2
import numpy as np
import base64
import json
import time
import logging
from threading import Thread
from agent import Agent, Device
from logger_config import setup_logger
import os  # Importación añadida para manejo de rutas de archivos

# Dimensiones máximas de tu sistema de visión
MAX_WIDTH = 1280
MAX_HEIGHT = 720

class VisionPosDevice:
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
        
        # Dimensiones estimadas del mosaico
        self.total_w = MAX_WIDTH
        self.total_h = MAX_HEIGHT
        self.H = np.load("homography_matrix.npy")
        self.boundaries_path = os.path.join(os.path.dirname(__file__), "tatami_config.json")
        self.boundaries = None
        self.load_arena_boundaries()
        
        # Parámetros de stitching
        self.offset_x = 0
        self.offset_y = 0

        # 2. Configuración ArUco (Heredado de pos_agent.py)
        # Usamos el diccionario 4x4 que tenías configurado
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Mejoras de detección para marcadores pequeños en cenital
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 25
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.aruco_params.minMarkerPerimeterRate = 0.01

        logger.info("VisionPosDevice inicializado correctamente. Cámaras y ArUco listos.")
    
    def load_arena_boundaries(self):
        """Intenta cargar los límites guardados localmente desde un archivo JSON."""
        if os.path.exists(self.boundaries_path):
            try:
                with open(self.boundaries_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                limites = data["boundaries"]
                logger.critical(f"Boundaries: {limites}")
                puntos = limites["points"]
                all_x = [p["x"] for p in puntos]
                all_y = [p["y"] for p in puntos]
                self.M_trans = np.array(data["M_pixel_to_real"])
                self.boundaries = {
                    "x_min": min(all_x),
                    "x_max": max(all_x),
                    "y_min": min(all_y),
                    "y_max": max(all_y)
                }
                logger.critical(f"Límites del tatami cargados localmente: {self.boundaries}")
                logger.critical(f"Matriz pixel2cm: {self.M_trans}")
            except Exception as e:
                logger.error(f"Error cargando los límites del tatami desde JSON: {e}")
        else:
            logger.warning(f"No se encontró el archivo de límites {self.boundaries_path}. Se usarán coordenadas crudas de píxeles.")
    
    def connect(self) -> None:
        logger.info(f"[INFO] Agente {self.agent.id} conectado y esperando video...")
    
    def start(self):
        self.running = True
        self.thread = Thread(target=self.loop, daemon=True)
        self.thread.start()
        logger.info("Bucle principal de Visión + Posición iniciado.")

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        self.cap_a.release()
        self.cap_b.release()
        cv2.destroyAllWindows()
        logger.info("Dispositivo detenido y cámaras liberadas.")

    def loop(self):
        try:
            while self.running:
                ret_a, frame_a = self.cap_a.read()
                ret_b, frame_b = self.cap_b.read()

                if not ret_a or not ret_b:
                    logger.warning("Fallo al capturar frame de alguna de las cámaras.")
                    time.sleep(0.01)
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

                    canvas_red = rescale_frame(canvas, MAX_WIDTH , MAX_HEIGHT)


                # --- 2. DETECCIÓN ARUCO LOCAL (Sin pasar por red) ---
                gray = cv2.cvtColor(canvas_red, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )

                robots_detected = []

                if ids is not None:
                    if self.gui:
                        cv2.aruco.drawDetectedMarkers(canvas_red, corners, ids)

                    # Iteramos y enviamos CADA robot por separado inmediatamente
                    for i in range(len(ids)):
                        marker_id = int(ids[i][0])
                        c = corners[i][0]
                        
                        # Calcular centro del marcador (X, Y) en pixeles
                        center_x = float(np.mean(c[:, 0]))
                        center_y = float(np.mean(c[:, 1]))
                        pt_pixel = np.array([[[center_x, center_y]]], dtype=np.float32)
                        pt_real = cv2.perspectiveTransform(pt_pixel, self.M_trans)[0][0]

                        real_x = pt_real[0] # Coordenada X en centímetros reales
                        real_y = pt_real[1] # Coordenada Y en centímetros reales

                        # Calcular orientación (theta) en radianes
                        # Calculamos theta directamente en el espacio de la imagen (Y hacia abajo)
                        v = c[1] - c[0]
                        theta = float(np.arctan2(v[1], v[0]))

                        # Estructura de payload individualizada
                        # (Ajusta los campos "x", "y" o el formato según lo que tuvieses en tu pos_agent original)
                        payload = {
                            "x": round(float(real_x), 2),
                            "y": round(float(real_y), 2),
                            "yaw": round(float(theta), 3),
                            "timestamp": time.time()
                        }
                        # Estructura de payload individualizada
                        # Enviar de forma individual. 
                        robot_id = int(marker_id)
                        target_topic = f"{robot_id}/pos"
                                    
                        # Opción B: Si tu sistema usaba tópicos dinámicos por ID (ej: robot/5/pos) desatenta esta línea:
                        self.agent.send(target_topic, payload)
                        
                        logger.debug(f"Enviada posición individual del Robot {robot_id}: {payload}")
                    
                # --- 4. VISUALIZACIÓN LOCAL (Si aplica) ---
                if self.gui:
                    cv2.imshow("Robotarium Cenital - Integrado", canvas_red )
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # Control de frecuencia (~20-25 FPS máximo para no ahogar la Pi)
                time.sleep(0.03)

        except Exception as e:
            logger.error(f"Error crítico en el bucle integrado: {e}", exc_info=True)
        finally:
            self.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gui', action='store_true', help="Ejecutar sin ventana de video local")
    args = parser.parse_args()

    fname = f"vision_pos_agent_log_{time.strftime('%Y%m%d_%H%M%S')}"
    logger = setup_logger(fname, console_level=logging.INFO)

    # Instanciamos el Agente centralizado
    agente_integrado = Agent(
        device_class=VisionPosDevice, 
        id="VisionPosSystem05", 
        ip="192.168.10.1",      # IP local de la Raspberry
        data_port=5559,
        hub_ip="192.168.10.1"   # IP del Hub
    )

    agente_integrado.device.gui = not args.no_gui
    
    # Arrancamos el loop
    agente_integrado.device.start()

    # Mantener vivo el hilo principal
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Apagando agente integrado...")
        agente_integrado.device.stop()