# ==================================================================================
# PROYECTO: Robotarium - Sistema de Visión Cenital
# ARCHIVO:  stitching.py
# FECHA:    14 de Abril, 2026
# OBJETIVO: Comprobación de la matriz de homografía (H) para la unión (stitching) de dos 
#           cámaras cenitales fijas. Permite alinear el espacio de trabajo de 
#           ambas cámaras en un único plano de coordenadas global.
#
# INSTRUCCIONES:
#   1. Carga la matriz  'homography_matrix.npy' y genera una imagen con la fusión de ambas cámaras
# ==================================================================================
import cv2
import numpy as np

# --- CONFIGURACIÓN ---
CAM_A = 5  # Índice o ruta de cámara A
CAM_B = 1  # Índice o ruta de cámara B
H_FILE = "homography_matrix.npy"

# Tamaño máximo para mostrar en pantalla
MAX_WIDTH = 1280
MAX_HEIGHT = 720

# --- CARGAR MATRIZ ---
H = np.load(H_FILE)

# --- CAPTURAR FRAMES ---
cap_a = cv2.VideoCapture(CAM_A)
cap_b = cv2.VideoCapture(CAM_B)

ret_a, frame_a = cap_a.read()
ret_b, frame_b = cap_b.read()
cap_a.release()
cap_b.release()

if not ret_a or not ret_b:
    print("Error al leer las cámaras")
    exit()

h_a, w_a, _ = frame_a.shape
h_b, w_b, _ = frame_b.shape

# --- CALCULAR OFFSET PARA EVITAR NEGROS ---
esquinas_b = np.array([[0,0],[w_b,0],[w_b,h_b],[0,h_b]], dtype=np.float32).reshape(-1,1,2)
esquinas_b_trans = cv2.perspectiveTransform(esquinas_b, H)

x_min = min(0, np.min(esquinas_b_trans[:,:,0]))
y_min = min(0, np.min(esquinas_b_trans[:,:,1]))
x_max = max(w_a, np.max(esquinas_b_trans[:,:,0]))
y_max = max(h_a, np.max(esquinas_b_trans[:,:,1]))

offset_x = -int(x_min)
offset_y = -int(y_min)
nuevo_w = int(x_max - x_min)
nuevo_h = int(y_max - y_min)

H_offset = H.copy()
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

canvas_red = rescale_frame(canvas, MAX_WIDTH, MAX_HEIGHT)

# --- MOSTRAR ---
cv2.imshow("Stitching Completo", canvas_red)
cv2.waitKey(0)
cv2.destroyAllWindows()