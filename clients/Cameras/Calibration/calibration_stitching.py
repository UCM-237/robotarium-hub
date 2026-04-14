# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  calibration_stitching.py
# FECHA:    14 de Abril, 2026
# OBJETIVO: Generar la matriz de homografía (H) para la unión (stitching) de dos 
#           cámaras cenitales fijas. Permite alinear el espacio de trabajo de 
#           ambas cámaras en un único plano de coordenadas global.
#
# INSTRUCCIONES:
#   1. Colocar 4 marcadores en la zona de solape del Robot Arena.
#   2. Ejecutar el script y seleccionar los mismos 4 puntos en ambas cámaras 
#      siguiendo estrictamente el mismo orden (ej. sentido horario).
#   3. La matriz resultante se guardará como 'homography_matrix.npy' para ser
#      utilizada por el Agente de Visión en tiempo real.
# ==================================================================================
import cv2
import numpy as np

# Listas para guardar los puntos seleccionados
points_cam_a = []
points_cam_b = []

def select_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if param['cam'] == 'A':
            points_cam_a.append((x, y))
            print(f"Cam A - Punto {len(points_cam_a)}: ({x}, {y})")
        else:
            points_cam_b.append((x, y))
            print(f"Cam B - Punto {len(points_cam_b)}: ({x}, {y})")

def calibrate():
    # 1. Capturar un frame de cada cámara (o cargar fotos guardadas)
    cap_a = cv2.VideoCapture(0)
    cap_b = cv2.VideoCapture(1)
    
    ret_a, frame_a = cap_a.read()
    ret_b, frame_b = cap_b.read()
    
    cv2.namedWindow("Camara A")
    cv2.setMouseCallback("Camara A", select_points, {'cam': 'A'})
    cv2.namedWindow("Camara B")
    cv2.setMouseCallback("Camara B", select_points, {'cam': 'B'})

    print("Haz clic en 4 puntos correspondientes en la zona de solape.")
    print("Orden: Superior-Izquierda, Superior-Derecha, Inferior-Izquierda, Inferior-Derecha")

    while len(points_cam_a) < 4 or len(points_cam_b) < 4:
        cv2.imshow("Camara A", frame_a)
        cv2.imshow("Camara B", frame_b)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 2. Calcular la Homografía
    pts_a = np.array(points_cam_a).astype(float)
    pts_b = np.array(points_cam_b).astype(float)
    
    # Buscamos la matriz que transforma puntos de B a las coordenadas de A
    H, status = cv2.findHomography(pts_b, pts_a)

    # 3. Guardar la matriz para usarla en el Agente de Visión
    np.save("homography_matrix.npy", H)
    print("Calibración completada y guardada como 'homography_matrix.npy'")

    # 4. Prueba rápida de unión (Stitching)
    h, w, _ = frame_a.shape
    # Creamos un lienzo doble de ancho
    canvas = cv2.warpPerspective(frame_b, H, (w * 2, h))
    canvas[0:h, 0:w] = frame_a
    
    cv2.imshow("Resultado Calibracion", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate()