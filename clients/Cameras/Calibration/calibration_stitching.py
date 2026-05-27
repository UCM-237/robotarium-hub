# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  stitching.py
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

# Tamaño máximo de ventana
MAX_WIDTH = 1280
MAX_HEIGHT = 720

def rescale_frame(frame, max_w, max_h):
    alto_original, ancho_original = frame.shape[:2]
    escala = min(max_w / ancho_original, max_h / alto_original)
    nuevo_ancho = int(ancho_original * escala)
    nuevo_alto  = int(alto_original * escala)
    frame_redimensionado = cv2.resize(frame, (nuevo_ancho, nuevo_alto), interpolation=cv2.INTER_AREA)
    return frame_redimensionado, escala

def select_points(event, x, y, flags, param):
    escala = param['scale']
    cam = param['cam']
    if event == cv2.EVENT_LBUTTONDOWN:
        x_original = int(x / escala)
        y_original = int(y / escala)
        if cam == 'A':
            points_cam_a.append((x_original, y_original))
            print(f"Cam A - Punto {len(points_cam_a)}: ({x_original}, {y_original})")
        else:
            points_cam_b.append((x_original, y_original))
            print(f"Cam B - Punto {len(points_cam_b)}: ({x_original}, {y_original})")

def draw_points(frame_red, points, scale):
    # Dibuja los puntos en la imagen escalada
    for idx, (x, y) in enumerate(points):
        x_scaled = int(x * scale)
        y_scaled = int(y * scale)
        cv2.circle(frame_red, (x_scaled, y_scaled), 5, (0, 0, 255), -1)
        cv2.putText(frame_red, str(idx+1), (x_scaled + 5, y_scaled - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

def calibrate():
    cap_a = cv2.VideoCapture(0)
    cap_b = cv2.VideoCapture(2)
    cap_a.set(cv2.CAP_PROP_FPS,10)
    cap_a.set(cv2.CAP_PROP_BUFFERSIZE,1)
    cap_a.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap_a.set(cv2.CAP_PROP_EXPOSURE, -7)
    cap_a.set(cv2.CAP_PROP_BRIGHTNESS, 100) # Un valor medio/bajo
        

    cap_b.set(cv2.CAP_PROP_FPS,10)
    cap_b.set(cv2.CAP_PROP_BUFFERSIZE,1)
    cap_b.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap_b.set(cv2.CAP_PROP_EXPOSURE, -7)
     # 3. Opcional: Bajar el brillo si la exposición no es suficiente
    cap_b.set(cv2.CAP_PROP_BRIGHTNESS, 100)
    
    cv2.namedWindow("Camara A")
    cv2.namedWindow("Camara B")
    
    print("Haz clic en 4 puntos correspondientes en la zona de solape.")
    print("Orden: Superior-Izquierda, Superior-Derecha, Inferior-Izquierda, Inferior-Derecha")
    
    escala_a = 1.0
    escala_b = 1.0

    while len(points_cam_a) < 4 or len(points_cam_b) < 4:
        ret_a, frame_a = cap_a.read()
        ret_b, frame_b = cap_b.read()
        if not ret_a or not ret_b:
            print("Error al leer la cámara")
            break

        frame_a_red, escala_a = rescale_frame(frame_a, MAX_WIDTH, MAX_HEIGHT)
        frame_b_red, escala_b = rescale_frame(frame_b, MAX_WIDTH, MAX_HEIGHT)

        # Dibuja puntos ya seleccionados
        draw_points(frame_a_red, points_cam_a, escala_a)
        draw_points(frame_b_red, points_cam_b, escala_b)

        cv2.setMouseCallback("Camara A", select_points, {'cam': 'A', 'scale': escala_a})
        cv2.setMouseCallback("Camara B", select_points, {'cam': 'B', 'scale': escala_b})

        cv2.imshow("Camara A", frame_a_red)
        cv2.imshow("Camara B", frame_b_red)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_a.release()
    cap_b.release()

    # Calcular homografía
    pts_a = np.array(points_cam_a).astype(float)
    pts_b = np.array(points_cam_b).astype(float)
    H, status = cv2.findHomography(pts_b, pts_a)
    np.save("homography_matrix.npy", H)
    print("Calibración completada y guardada como 'homography_matrix.npy'")

    # Prueba rápida de unión (Stitching)
    h, w, _ = frame_a.shape
    canvas = cv2.warpPerspective(frame_b, H, (w * 2, h))
    canvas[0:h, 0:w] = frame_a
    cv2.imshow("Resultado Calibracion", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate()