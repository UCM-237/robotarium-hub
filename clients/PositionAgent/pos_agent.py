import cv2
import numpy as np
import base64
import json
from agent import Agent, Device
import string
import time

class ArucoDevice:
    def __init__(self, agent: Agent) -> None:
        print("Inicializando ArucoDevice")
        self.agent = agent
        self.window_name = "Robotarium - Recepcion Vision"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 1. Configurar el diccionario ArUco y los parámetros de detección
        # Usamos el diccionario 6x6 que es el estándar para robótica
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.H = np.load("homography_matrix.npy")
        # 2. Configuración de los tiempos de envio
        self.Tdraw=1 # Se dibuja cada 2s
        self.current_frame = None
        self.last_corners = None
        self.last_ids = None
        self.running = True
        # 3. --- Configuración del nuevo sistema ---
        self.WIDTH_ARENA = 419  # cm
        self.HEIGHT_ARENA = 140 # cm
        #TO REVIEW
        self.OFFSET_X=854.14
        self. OFFSET_Y=434.92
        
        # Escala (Valor_Máximo_Deseado / Valor_Máximo_Raw_Detectado)
        # X_raw_max (1191.39)
        # Y_raw_max (basado en robot 4 arriba) approx 358
        self.SCALE_X = 419.0 / 1191.39
        self.SCALE_Y = -140.0 / 358.0 # Ajuste estimado según robot 4
        self.last_draw_time=0
        
        self.frame_to_show=None

    def connect(self) -> None:
        print(f"[INFO] Agente {self.agent.id} conectado y esperando video...")

    def on_data(self, topic: str, message: str) -> None:
        """
        Este método es llamado automáticamente por agent.py 
        cuando llega un mensaje al tópico suscrito.
        """
        # ---------------------------------------
        if topic == "vision/stitched":
            try:
                # 1. Convertir el string JSON a diccionario
                data = json.loads(message)
                
                # 2. Extraer la imagen en Base64 y decodificarla
                # Basado en el payload que envía tu vision_agent.py
                img_b64 = data['image']
                img_bytes = base64.b64decode(img_b64)
                
                # 3. Convertir bytes a imagen de OpenCV
                np_array = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    # 2. DETECCIÓN DE ARUCOS
                    # corners: lista de esquinas de los marcadores detectados
                    # ids: identificadores de cada marcador
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        frame, 
                        self.aruco_dict, 
                        parameters=self.aruco_params
                    )
                    if ids is None:
                        print("No markers detected on frame")
                    else:
                        ids_flat = ids.flatten()
                        self.current_frame = frame
                        self.last_corners = corners
                        self.last_ids = ids
                        for i, corner in enumerate(corners):
                            # 1. Obtener puntos clave del marcador en píxeles (u, v)
                            c = corner[0] # Esquinas: [0]=atrás-izq, [1]=atrás-der, [2]=alante-der, [3]=alante-izq (aprox)
                            
                            # Centro del marcador en píxeles
                            pixel_center = np.mean(c, axis=0)
                            
                            # Punto frontal (media de las dos esquinas delanteras para definir el "morro")
                            pixel_front = np.mean([c[1], c[2]], axis=0) 

                            # 2. Transformar puntos de Píxeles -> Mundo Real usando la Homografía
                            # cv2.perspectiveTransform requiere un array de forma (N, 1, 2)
                            # Creamos un array de float32 con forma (2, 1, 2)
                            pts = np.array([pixel_center, pixel_front], dtype='float32').reshape(-1, 1, 2)
                            # 3. Aplicar la transformación
                            # Si self.H es tu matriz 3x3
                            try:
                                real_pts = cv2.perspectiveTransform(pts, self.H)
                                
                                # Extraer los resultados (ahora tienen forma 2, 1, 2)
                                real_x, real_y = real_pts[0][0]
                                front_x, front_y = real_pts[1][0]

                                # 4. Calcular ángulo
                                x_raw, y_raw = real_pts[0][0]
                                fx_raw, fy_raw = real_pts[1][0]
                                # 2. Re-mapeo al nuevo origen (Esquina inferior derecha)
                                # Invertimos los ejes restando del máximo
                                x_new = (x_raw-self.OFFSET_X)*self.SCALE_X
                                y_new = (y_raw-self.OFFSET_Y)*self.SCALE_Y
                        
                                # 3. Cálculo del Yaw en el nuevo sistema
                                # Calculamos el frente nuevo también para obtener el vector dirección
                                fx_new = (fx_raw-self.OFFSET_Y)*self.SCALE_X
                                fy_new = (fy_raw-self.OFFSET_Y)*self.SCALE_Y
                                
                                # 2. Calcular el ángulo en PÍXELES (aquí nunca te dará 0)
                                # Invertimos el eje Y de la imagen porque en OpenCV crece hacia abajo
                                dx_px = pixel_front[0] - pixel_center[0]
                                dy_px = -(pixel_front[1] - pixel_center[1]) 

                                yaw_new = np.arctan2(dy_px, dx_px)
                                
                                print(f"ID {ids[i][0]}: X={x_new:.2f}, Y={y_new:.2f}, Th={yaw_new:.2f}")
                                # 4. (Opcional) Publicar para el servidor/robots
                                # 4. ENVÍO DE DATOS
                                robot_id = int(ids[i][0])
                                target_topic = f"{robot_id}/pos"
                            
                                payload = {
                                    "x": round(float(x_new), 2),
                                    "y": round(float(y_new), 2),
                                    "yaw": round(float(yaw_new), 3)
                                }
                                self.agent.send(target_topic, json.dumps(payload))
                                print(f"Mensaje {json.dumps(payload)} enviado en topic {target_topic}")

                            except cv2.error as e:
                                print(f"Error en la transformación: {e}")


                        
 
                    
            except Exception as e:
                print(f"[ERROR] Error al procesar frame: {e}")

  

    def run(self):
        """
        Este método corre en el hilo principal y gestiona la visualización.
        """
        
        print(f"[INFO] {self.agent.id} visualizando...")
        try:
            while self.running:
                # Si hay un frame nuevo, lo procesamos para mostrar
                if self.current_frame is not None:
                    current_time=time.time()
                    if (current_time-self.last_draw_time)>self.Tdraw:
                        # Creamos una copia local para no interferir con on_data
                        display_frame = self.current_frame.copy()
                        
                        # Dibujamos los últimos marcadores conocidos si existen
                        if self.last_ids is not None:
                            cv2.aruco.drawDetectedMarkers(display_frame, self.last_corners, self.last_ids)
                        
                        cv2.imshow(self.window_name, display_frame)
                    
                    # El waitKey(1) permite que la ventana responda y se refresque
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        finally:
            cv2.destroyAllWindows()

# --- LANZAMIENTO DEL AGENTE ---
if __name__ == "__main__":
    # IMPORTANTE: En agent.py, asegúrate de añadir la suscripción 
    # al tópico 'vision/stitched' en el método listen()
    
    aruco_agent = Agent(
        device_class=ArucoDevice,
        id="ArucoTracker",
        ip="192.168.10.1",        # Tu IP
        hub_ip="192.168.10.1",  # IP del Hub
        data_port = 5560
    )
    
    # Iniciamos el bucle pasivo
    #aruco_agent.device.connect()
    aruco_agent.device.run()