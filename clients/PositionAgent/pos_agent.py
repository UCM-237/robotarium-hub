import cv2
import numpy as np
import base64
import json
from agent import Agent, Device
import string
import time
import argparse
from logger_config import setup_logger
import logging
import queue
import threading



class ArucoDevice:
    def __init__(self, agent: Agent) -> None:
        logger.info("Inicializando ArucoDevice")
        self.agent = agent
        self.window_name = "Robotarium - Recepcion Vision"
        self.missed_frames = 0
        #cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 1. Configurar el diccionario ArUco y los parámetros de detección
        # Usamos el diccionario 6x6 que es el estándar para robótica
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # --- MEJORAS DE DETECCIÓN ---
        # Reduce el tamaño de la ventana de umbralización para detectar marcadores pequeños
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 25
        self.aruco_params.adaptiveThreshWinSizeStep =5
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.markerBorderBits =1 
        self.aruco_params.adaptiveThreshConstant =7
        self.aruco_params.polygonalApproxAccuracyRate=0.05
        self.aruco_params.perspectiveRemovePixelPerCell = 4
        self.aruco_params.minMarkerDistanceRate = 0.05
        # Aumenta la precisión de las esquinas (Crucial para el cálculo de Yaw)
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.errorCorrectionRate = 0.8
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        self.H = np.load("homography_matrix.npy")
        # 2. Configuración de los tiempos de envio
        self.Tdraw=0.1 # Se dibuja cada 2s
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
        # Creamos una cola para procesar las imágenes fuera del hilo de MQTT
        self.image_queue = queue.Queue(maxsize=3) 
        self.consecutive_corruptions = 0
        self.MAX_CORRUPTIONS_ALLOWED = 15  # Umbral para reiniciar la red
        # Lanzamos un hilo dedicado exclusivamente a procesar las imágenes
        self.process_thread = threading.Thread(target=self._image_processing_loop, daemon=True)
        self.process_thread.start()

    def connect(self) -> None:
        logger.info(f"[INFO] Agente {self.agent.id} conectado y esperando video...")
        #self.agent.setup_subscriptions() # Configuramos las suscripciones al Hub
        #logger.info(f"[INFO] Agente {self.agent.id} suscrito a topics")

  
    def on_data(self, topic: str, message: str) -> None:
        """
        Este método es llamado automáticamente por agent.py 
        cuando llega un mensaje al tópico suscrito.
        """
        # ---------------------------------------
        logger.debug(f"Dato recibido en tópico {topic}")
        if topic == "vision/stitched":
            try:
                if isinstance(message, bytes):
                    msg_str = message.decode('utf-8', errors='ignore').strip()
                else:
                    msg_str = str(message).strip()

                if "distance" in msg_str or "agent" in msg_str or not msg_str.startswith('{'):
                    self.consecutive_corruptions += 1
                    logger.warning(f"[ALERTA HUB] Corrupción detectada ({self.consecutive_corruptions}/{self.MAX_CORRUPTIONS_ALLOWED}). Mensaje ignorado.")
                    
                    # SI PASAMOS EL LÍMITE, DETECTAMOS QUE EL HUB SE QUEDÓ ENGANCHADO Y REINICIAMOS
                    if self.consecutive_corruptions >= self.MAX_CORRUPTIONS_ALLOWED:
                        logger.critical("¡RED DESALINEADA DE FORMA PERMANENTE! Forzando reinicio del socket del Agente...")
                        
                        # Reseteamos contadores
                        self.consecutive_corruptions = 0
                        
                        # Vaciamos la cola de imágenes por si acaso hay basura
                        while not self.image_queue.empty():
                            try:
                                self.image_queue.get_nowait()
                                self.image_queue.task_done()
                            except queue.Empty:
                                break
                # INTENTO DE RECONEXIÓN AUTOMÁTICA DEL AGENTE
                        # Dependiendo de cómo esté hecho vuestro 'agent.py', esto cierra y reabre los sockets:
                        try:
                            if hasattr(self.agent, 'disconnect') and hasattr(self.agent, 'connect'):
                                self.agent.disconnect()
                                time.sleep(0.5)
                                self.agent.connect()
                            elif hasattr(self.agent, 'socket') and hasattr(self.agent, 'listen'):
                                # Si usa sockets de ZeroMQ expuestos directamente, se pueden cerrar
                                logger.info("Cerrando y levantando hilo de escucha...")
                                # Nota: Si vuestro agent.py no tiene disconnect(), una alternativa radical pero 
                                # 100% efectiva es forzar la salida del script para que vuestro gestor de procesos 
                                # (o un bucle bash) lo reinicie instantáneamente:
                                # import os; os._exit(1)
                        except Exception as re_err:
                            logger.error(f"Error al intentar reconectar el agente: {re_err}")
                            # Si no se puede reconectar programáticamente, cerramos el script para que el sistema lo reviva
                            import os; os._exit(1)
                            
                    return 

                # Si el mensaje es correcto, ponemos a cero el contador de fallos
                self.consecutive_corruptions = 0
                # Encolar frame de manera segura
                self.image_queue.put_nowait(msg_str)

            except queue.Full:
                self.missed_frames += 1
                if self.missed_frames % 30 == 0:
                    logger.warning(f"[WARNING] Cola de imágenes llena. Frames descartados: {self.missed_frames}")
            except Exception as e:
                logger.error(f"Error al filtrar/encolar frame: {e}")

    def _image_processing_loop(self):

        """Hilo aislado. Si aquí explota OpenCV, la red MQTT sigue funcionando intacta."""
        while True:
            try:
                try:
                    payload = self.image_queue.get(timeout=2.0)
                except queue.Empty:
                    continue
                    # 1. Convertir el string JSON a diccionario
                try:
                    data = json.loads(payload)
                    img_b64=data.get("image")
                except Exception as json_error:
                    logger.error(f"[ERROR] Error al decodificar JSON: {payload}")
                    self.image_queue.task_done()
                    continue
                if img_b64:
                    try: 
                    # 2. Validación de Base64: nos aseguramos de que no termine en basura
                        # Añadimos padding por si el Hub lo cortó a la mitad
                        missing_padding = len(img_b64) % 4
                        if missing_padding:
                            img_b64 += '=' * (4 - missing_padding)

                        img_bytes = base64.b64decode(img_b64)
                        np_arr = np.frombuffer(img_bytes, np.uint8)
                            
                        # 3. Evitamos que imdecode se quede colgado con arrays mal formados
                        if np_arr.size == 0:
                            self.image_queue.task_done()
                            continue
                                
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if frame is not None and frame.size > 0:
                                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                        # Aplicamos una ecualización de histograma para resaltar los bordes
                        # Esto ayuda mucho si la iluminación es pobre
                        #gray = cv2.equalizeHist(gray)
                        if frame is not None:
                            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            # 2. DETECCIÓN DE ARUCOS
                            # corners: lista de esquinas de los marcadores detectados
                            # ids: identificadores de cada marcador
                            #Justo antes de detectMarkers, aplica un umbral manual para testear
                            # Esto te permitirá ver si el ArUco se está "emborronando"
                            _, testing_thresh = cv2.threshold(gray, 150, 180, cv2.THRESH_BINARY)
                            #cv2.imshow("Test Umbral", testing_thresh) # Si aquí el ArUco se ve todo negro o todo blanco, ahí está el problema
                            corners, ids, rejected = cv2.aruco.detectMarkers(
                                gray, 
                                self.aruco_dict, 
                                parameters=self.aruco_params
                            )
                            logger.debug(f"Marcadores detectados: {len(ids) if ids is not None else 0}")
                            # --- TRUCO DE DEBUG ---

                            # Dibuja en ROJO los cuadros que el algoritmo VIÓ pero DESCARTÓ por no ser ArUcos válidos
                            
                            #cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(0, 0, 255))
                            cv2.aruco.drawDetectedMarkers(frame,corners,ids,borderColor=(0,255,0))
                            cv2.imshow("debug_window",frame)
                            cv2.waitKey(1)
                            if ids is None:
                                logger.warning("No markers detected on frame")
                                logger.debug(f"Marcadores rechazados: {len(rejected)}")
                                self.missed_frames+=1
                                if self.missed_frames %10 ==0:
                                    logger.warning(f"Ojo {self.missed_frames} frames sin ver robots")
                            else:
                                self.missed_frames = 1
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
                                        
                                        logger.info(f"ID {ids[i][0]}: X={x_new:.2f}, Y={y_new:.2f}, Th={yaw_new:.2f}")
                                        # 4. (Opcional) Publicar para el servidor/robots
                                        # 4. ENVÍO DE DATOS
                                        robot_id = int(ids[i][0])
                                        target_topic = f"{robot_id}/pos"
                                    
                                        payload = {
                                            "x": round(float(x_new), 2),
                                            "y": round(float(y_new), 2),
                                            "yaw": round(float(yaw_new), 3),
                                            "timestamp": time.time()
                                        }
                                        try:
                                            self.agent.send(target_topic,payload)
                                            logger.debug(f"Enviado -> {target_topic}: {payload}")
                                        except Exception as send_error:
                                            logger.error(f"[ERROR] Error al enviar datos a {target_topic}: {send_error}")
                                    except cv2.error as e:
                                        logger.error(f"Error en la transformación: {e}")
                    except cv2.error as cv_err:
                        logger.error(f"Error nativo de OpenCV (evitado bloqueo): {cv_err}")
                    except Exception as img_err:
                        logger.error(f"Error procesando estructura de imagen: {img_err}")
                    self.image_queue.task_done()
            except Exception as e:
                logger.error(f"[ERROR] Error crítico en el bucle principal de imágenes: {e}", exc_info=True)
                # Pequeño respiro por si hay un error masivo continuo para no saturar la CPU
                time.sleep(0.1)    
        
                    
    def run(self, gui=True):
        pass

# --- LANZAMIENTO DEL AGENTE ---
if __name__ == "__main__":
    # IMPORTANTE: En agent.py, asegúrate de añadir la suscripción 
    # al tópico 'vision/stitched' en el método listen()
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gui', action='store_true', help="Ejecutar sin ventana de video")
    args = parser.parse_args()
    # 1. Creamos un manejador de consola (StreamHandler)
    fname=f"pos_agent_log_{time.strftime('%Y%m%d_%H%M%S')}"
    logger = setup_logger(fname,console_level=logging.WARNING)
    logger.propagate = False
    time.sleep(1)  
    aruco_agent = Agent(
        device_class=ArucoDevice,
        id="ArucoTracker",
        ip="192.168.10.1",        # Tu IP
        hub_ip="192.168.10.1",  # IP del Hub
        data_port = 5590
    )
    topic=b'vision/stitched'
    aruco_agent.setup_subscriptions(topic)
    # Iniciamos el bucle pasivo
    #aruco_agent.device.connect()
    aruco_agent.device.run(gui=not args.no_gui)
