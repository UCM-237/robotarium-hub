# -*- coding: UTF-8 -*-
#!/bin/python3

import cv2
import numpy as np
import json
import os

# Configuración del tamaño del canvas (debe coincidir con tu sistema de visión)
MAX_WIDTH = 1280
MAX_HEIGHT = 720

class TatamiCalibrator:
    def __init__(self):
        self.points = []
        self.homography_path = "homography_matrix.npy"
        self.config_path = "tatami_config.json"
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
        # Intentar cargar la homografía
        if os.path.exists(self.homography_path):
            self.H = np.load(self.homography_path)
            print("[OK] Matriz de homografía cargada correctamente.")
        else:
            print("[ERROR] No se encontró 'homography_matrix.npy'. Asegúrate de calibrar las cámaras primero.")
            self.H = np.eye(3) # Matriz identidad por defecto si no existe

        # Inicializar capturas (Mismos índices que en tu VisionPosDevice)
        

    def get_stitched_frame(self):
        ret_a, frame_a = self.cap_a.read()
        ret_b, frame_b = self.cap_b.read()

        if not ret_a or not ret_b:
            return None

        h_a, w_a, _ = frame_a.shape
        h_b, w_b, _ = frame_b.shape

        # Calcular offset basándose en la homografía para evitar zonas negras
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

        canvas = np.zeros((nuevo_h, nuevo_w, 3), dtype=np.uint8)
        canvas[offset_y:offset_y+h_a, offset_x:offset_x+w_a] = frame_a

        frame_b_warped = cv2.warpPerspective(frame_b, H_offset, (nuevo_w, nuevo_h))
        mask = frame_b_warped > 0
        canvas[mask] = frame_b_warped[mask]

        # Redimensionar al tamaño estándar de trabajo
        alto, ancho = canvas.shape[:2]
        escala = min(MAX_WIDTH / ancho, MAX_HEIGHT / alto)
        return cv2.resize(canvas, (int(ancho * escala), int(alto * escala)), interpolation=cv2.INTER_AREA)

    def click_event(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < 4:
                self.points.append((x, y))
                print(f"Punto {len(self.points)} registrado: ({x}, {y})")

    def run_calibration(self):
        print("\n--- INICIANDO CALIBRACIÓN DEL TATAMI ---")
        print("Instrucciones:")
        print("1. Haz clic en las 4 esquinas del tatami de goma negra en el siguiente orden:")
        print("   [1] Arriba-Izquierda  ->  [2] Arriba-Derecha  ->  [3] Abajo-Derecha  ->  [4] Abajo-Izquierda")
        print("2. Presiona 'r' para reiniciar la selección si te equivocas.")
        print("3. Presiona 'q' para salir sin guardar.")
        print("4. Una vez marcados los 4 puntos, introduce en la consola las dimensiones reales.")

        cv2.namedWindow("Calibracion Tatami")
        cv2.setMouseCallback("Calibracion Tatami", self.click_event)

        while True:
            frame = self.get_stitched_frame()
            if frame is None:
                print("[WARNING] Esperando frames de las cámaras...")
                continue

            # Dibujar puntos seleccionados y líneas de conexión
            for i, pt in enumerate(self.points):
                cv2.circle(frame, pt, 5, (0, 255, 0), -1)
                cv2.putText(frame, f"P{i+1}", (pt[0]+10, pt[1]-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            if len(self.points) > 1:
                for i in range(len(self.points) - 1):
                    cv2.line(frame, self.points[i], self.points[i+1], (0, 255, 0), 2)
            if len(self.points) == 4:
                cv2.line(frame, self.points[3], self.points[0], (0, 255, 0), 2)

            cv2.imshow("Calibracion Tatami", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('r'):
                self.points = []
                print("Selección reiniciada.")
            elif key == ord('q'):
                print("Calibración cancelada.")
                break

            if len(self.points) == 4:
                cv2.imshow("Calibracion Tatami", frame)
                cv2.waitKey(500) # Pausa para ver el cuadrilátero completo
                break

        cv2.destroyAllWindows()
        self.cap_a.release()
        self.cap_b.release()

        if len(self.points) == 4:
            self.save_calibration()

    def save_calibration(self):
        print("\n--- DIMENSIONES REALES DEL TATAMI ---")
        try:
            real_w = float(input("Introduce el ANCHO real del tatami en cm (ej. 300.0): "))
            real_h = float(input("Introduce el ALTO real del tatami en cm (ej. 200.0): "))
        except ValueError:
            print("[ERROR] Entrada no válida. Se usarán valores por defecto (300x200 cm).")
            real_w, real_h = 300.0, 200.0

        # Mapeamos los 4 puntos de píxeles a coordenadas reales (cm)
        # P1 -> (0, 0)
        # P2 -> (real_w, 0)
        # P3 -> (real_w, real_h)
        # P4 -> (0, real_h)
        pts_pixel = np.array(self.points, dtype=np.float32)
        pts_real = np.array([
            [0, 0],
            [real_w, 0],
            [real_w, real_h],
            [0, real_h]
        ], dtype=np.float32)

        # Calculamos la matriz para transformar píxeles de cámara a coordenadas reales del tatami (cm)
        M_pixel_to_real, _ = cv2.findHomography(pts_pixel, pts_real)

        # Definir los límites del Tatami en centímetros (para que consuma bouncer.py)
        # Al estar mapeado a coordenadas reales, los límites irán desde 0 hasta real_w y real_h.
        boundaries_json = {
            "points": [
                {"x": 0.0, "y": 0.0},
                {"x": real_w, "y": 0.0},
                {"x": real_w, "y": real_h},
                {"x": 0.0, "y": real_h}
            ]
        }

        # Guardar archivo de configuración
        config_data = {
            "pixel_points": self.points,
            "real_dimensions": {"width_cm": real_w, "height_cm": real_h},
            "boundaries": boundaries_json,
            "M_pixel_to_real": M_pixel_to_real.tolist()
        }

        with open(self.config_path, "w") as f:
            json.dump(config_data, f, indent=4)

        print(f"\n[OK] Calibración completada con éxito.")
        print(f"Archivo de configuración guardado en: {self.config_path}")
        print(f"Límites del Tatami guardados: Ancho = {real_w}cm, Alto = {real_h}cm")

if __name__ == "__main__":
    calibrator = TatamiCalibrator()
    calibrator.run_calibration()