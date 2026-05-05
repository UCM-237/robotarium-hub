import cv2
import numpy as np
import base64
import json
from agent import Agent, Device
import string
import time
import argparse


class ArenaDevice:
    def __init__(self, agent: Agent) -> None:
        print("Inicializando ArenaDevice")
        self.agent = agent
        self.window_name = "Robotarium - Recepcion Vision"
        self.gui = True # Por defecto True, se cambiará desde el main
        self.running = True
        #cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 1. Configurar el diccionario ArUco y los parámetros de detección
        # Usamos el diccionario 6x6 que es el estándar para robótica
        self.H = np.load("homography_matrix.npy")
        # --- Configuración del nuevo sistema ---
        self.WIDTH_ARENA = 419  # cm
        self.HEIGHT_ARENA = 140 # cm
        self.OFFSET_X=854.14
        self.OFFSET_Y=434.92
        
        # Escala (Valor_Máximo_Deseado / Valor_Máximo_Raw_Detectado)
        # X_raw_max (1191.39)
        # Y_raw_max (basado en robot 4 arriba) approx 358
        self.SCALE_X = 419.0 / 1191.39
        self.SCALE_Y = -140.0 / 358.0 # Ajuste estimado según robot 4
        self.REAL_AREA=self.WIDTH_ARENA*self.HEIGHT_ARENA
        self.AREA_TOLERANCE=0.001
        self.H_inv = np.linalg.inv(self.H) # Pre-calculamos la inversa para dibujar

        # --- CONTROL DE PUBLICACIÓN ---
        self.Tenvio = 60.0  # Publicar cada 2 segundos
        self.last_publish_time = 0
        self.last_draw_time=0
        self.DrawTime=10.0 # Dibujar cada 10s
        self.last_valid_pts = None # Memoria para estabilidad

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
                    # 2. Procesamiento para detectar el tatami
                    # 1. RECORTAR RUIDO EXTERIOR (ROI)
                    # Si la pared está en los bordes, ignoramos un margen de píxeles
                    
                    margin = 40 # píxeles de margen para ignorar paredes
                    
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # El desenfoque es CRÍTICO para Canny, elimina ruido de píxeles sueltos
                    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

                    # 2. CANNY EDGE DETECTION
                    # Umbrales: 50 (mínimo) y 150 (máximo). Ajusta si ves demasiadas o pocas líneas.
                    edged = cv2.Canny(blurred, 20, 180)
                    h, w = edged.shape[:2]
                    edged[0:margin,:]=0
                    edged[h-margin:h,:]=0

                    # 3. DILATACIÓN (Engrosamos los bordes detectados para cerrar posibles huecos en la cinta)
                    kernel = np.ones((5, 5), np.uint8)
                    dilated = cv2.dilate(edged, kernel, iterations=1)
                    #cv2.imshow("Dilated borders", dilated)
                    #cv2.waitKey(1)
                        # 4. BUSCAR EL RECTÁNGULO DEL TATAMI
                    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    valid_tatami = None
                    
                    if contours:
                        # 1. Filtrar por área estimada antes de transformar
                        # Solo miramos contornos que tengan un tamaño razonable en la imagen
                        # para ignorar marcadores ArUco pequeños.
                        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
                        clean_pts=[]
                        for c in sorted_contours:
                            peri = cv2.arcLength(c, True)
                            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                            #print(f"Perimetro del contorno {peri}. Numero de lados {len(approx)}")

                            # 2. Solo nos interesan polígonos de 4 lados (rectángulos)
                            if len(approx) >= 4:
                                # Transformamos los 4 puntos al mundo real (cm)
                                pts_px = approx.astype('float32').reshape(-1, 1, 2)


                                transformed = cv2.perspectiveTransform(pts_px, self.H)
                                
                                pts_cm = []
                                pts_real = []
                                for p in transformed:
                                    xr, yr = p[0]
                                    # Convertimos de float32 de numpy a float nativo de Python
                                    x_cm = float(xr - self.OFFSET_X)*self.SCALE_X
                                    y_cm = float((yr - self.OFFSET_Y))*self.SCALE_Y
                                    # Estructura limpia para JSON
                                    # --- NUEVO FILTRO DE COORDENADAS ---
                                    # Ignoramos puntos que estén muy fuera de los límites lógicos (ruido de cámara)
                                    if x_cm > -10 and y_cm > -10:
                                        pts_real.append({"x": round(x_cm, 2), "y": round(y_cm, 2)})
                                        pts_cm.append([x_cm, y_cm])    
                                
                                if len(pts_cm) < 4:
                                    print(f"Puntos detectados insuficientes tras filtro de coordenadas: {pts_cm}")
                                    continue # Necesitamos al menos 4 puntos para un rectángulo válido

                                # --- FILTRO DE PIQUITOS ---
                                # Fusionamos puntos que estén a menos de 20cm
                                clean_pts = self.filter_close_points(pts_cm)
                                print(f"Puntos tras filtro {clean_pts}")
                                # Convertimos la lista a un numpy array con el tipo correcto (float32)
                                contour_array = np.array(clean_pts, dtype='float32')

                                # Ahora cv2.contourArea no fallará
                                detected_area = cv2.contourArea(contour_array)
                                
                                # ¿Se parece al área de 419x140?
                                area_diff = abs(detected_area - self.REAL_AREA) / self.REAL_AREA
                                print(f"Detected area {detected_area}; area diff {area_diff}")
                                if detected_area>10000:
                                    # ¡HEMOS ENCONTRADO EL TATAMI REAL!
                                    # Los marcadores ArUco tienen un área de ~100-200 cm2, 
                                    # jamás pasarán este filtro de 58,000 cm2.
                                    break
                                else:
                                    clean_pts=[]                        
                    if clean_pts:
                        # Ordenar puntos: Superior-Izquierda, Superior-Derecha, Inf-Der, Inf-Izq
                        # Esto facilita mucho el cálculo de distancias a las "paredes"
                        pts_array = np.array([[p[0], p[1]] for p in clean_pts])
                        
                        # Suma y diferencia para encontrar las esquinas
                        s = pts_array.sum(axis=1)
                        d = np.diff(pts_array, axis=1)
                        
                        ordered_pts = [
                            pts_array[np.argmin(s)], # Sup-Izq (x+y mínimo)
                            pts_array[np.argmin(d)], # Sup-Der (y-x mínimo)
                            pts_array[np.argmax(s)], # Inf-Der (x+y máximo)
                            pts_array[np.argmax(d)]  # Inf-Izq (y-x máximo)
                        ]
                    
                        final_pts = [{"x": round(float(p[0]), 2), "y": round(float(p[1]), 2)} for p in ordered_pts]
                        self.last_valid_pts=final_pts
                        # Publicar
                        # Envio el mensaje pero solo cada tiempo Tenvio para no saturar
                        current_time=time.time()
                        target_topic=f"arena/boundaries"
                        if (current_time-self.last_publish_time) >self.Tenvio:
                            if self.last_valid_pts:
                                payload = {
                                    "points": final_pts
                                }
                                self.agent.send(target_topic, json.dumps(payload))
                                self.last_publish_time=current_time
                                print(f"[INFO] Límites publicados {final_pts} (Frecuencia: {self.Tenvio}s)")
                      
                        # Opcional: Visualización para debug
                        if self.gui:
                            # Solo ejecutamos el dibujo y el imshow si gui es True
                            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
                            # Feedback visual de los índices para debug
                            for idx, p in enumerate(ordered_pts):
                                cv2.putText(frame, str(idx), (int(approx[idx][0][0]), int(approx[idx][0][1])), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            cv2.imshow("Deteccion Arena", frame)
                            cv2.waitKey(1)
            except Exception as e:
                print(f"[ERROR] Error al procesar frame: {e}")
    
    # --- FUNCIÓN AUXILIAR DE DIBUJO ---
    def draw_real_points(self, image, points_cm, color=(0, 255, 0), thickness=3):
        """ Re-proyecta puntos de CM a Píxeles y los dibuja sobre la imagen. """
        if not points_cm or len(points_cm) < 3:
            return image

        # 1. Deshacer calibración manual (volver a valores RAW de H)
        pts_raw = []
        for p in points_cm:
            xr = p['x'] + self.OFFSET_X
            yr = self.OFFSET_Y - p['y'] # Deshacer abs() y resta
            pts_raw.append([xr, yr])
        
        # 2. Re-proyectar a Píxeles usando la matriz inversa de H
        pts_raw_np = np.array(pts_raw, dtype='float32').reshape(-1, 1, 2)
        pts_px = cv2.perspectiveTransform(pts_raw_np, self.H_inv)
        
        # 3. Dibujar el polígono cerrado
        pts_px_int = pts_px.astype(int).reshape((-1, 1, 2))
        cv2.polylines(image, [pts_px_int], True, color, thickness)
        
        # Opcional: Dibujar los vértices para ver dónde están exactamente
        for p in pts_px_int:
            cv2.circle(image, tuple(p[0]), 5, (255, 0, 0), -1)
            
        return image
    
    def show_frame(self, frame):
        cv2.imshow(self.window_name, frame)
        # IMPORTANTE: waitKey es vital para que la ventana se refresque
        cv2.waitKey(1)

    def run(self):
        while self.running:
            time.sleep(1) # Duerme un segundo completo, ya que el trabajo real ocurre en on_data

    def filter_close_points(self,points, min_dist=130.0):
        """
        Elimina puntos que estén a menos de min_dist (cm) del punto anterior.
        """
        if not points:
            return []
        
        filtered = [points[0]]
       
        for i in range(1, len(points)):
            # Calcular distancia entre el punto actual y el último aceptado
            p1 = points[i]
            p2 = filtered[-1]
           
            dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
            if dist > min_dist:
                filtered.append(p1)
                
        # Comprobar también el último con el primero (cierre del polígono)
        if len(filtered) > 1:
            p1 = filtered[0]
            p2 = filtered[-1]
            dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            if dist < min_dist:
                filtered.pop() # Eliminar el último si está muy cerca del primero
            
        return filtered

# --- LANZAMIENTO DEL AGENTE ---
if __name__ == "__main__":
    # IMPORTANTE: En agent.py, asegúrate de añadir la suscripción 
    # al tópico 'vision/stitched' en el método listen()
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gui', action='store_true', help="Modo consola")
    args = parser.parse_args()

    arena_agent = Agent(
        device_class=ArenaDevice,
        id="RobotArena",
        ip="192.168.10.1",        # Tu IP
        hub_ip="192.168.10.1",  # IP del Hub
        data_port = 5561
    )
    arena_agent.device.gui = not args.no_gui # Seteamos el modo
    arena_agent.device.run()

    # Iniciamos el bucle pasivo
    #aruco_agent.device.connect()
    #aruco_agent.device.run()
