import numpy as np

class Segmento:
    def __init__(self, pi, pf):
        self.pi = pi
        self.pf = pf

def distancia(x, y, seg):
    # proyección del punto en la línea que define el segmento (producto escalar)
    proy = np.dot((seg.pf - seg.pi), np.array([x, y]) - seg.pi) / np.linalg.norm(seg.pf - seg.pi)
    # posición del punto más cercano respecto a pf
    L = np.linalg.norm(seg.pf - seg.pi)  # longitud del segmento
    
    # hay tres casos:
    if proy <= 0:
        # A) la distancia proyectada es negativa, entonces el punto más cercano a la recta
        # está fuera del segmento y el punto más cercano en el segmento es pi
        pc = seg.pi
    elif proy >= L:
        # B) en este caso estoy el punto más cercano a la recta
        # está fuera del segmento y el punto más cercano en el segmento es pf
        pc = seg.pf
    else:
        # C) el punto más cercano a la recta está dentro del segmento (caso por
        # defecto) primero saco el punto más cercano
        pc = seg.pi + proy * (seg.pf - seg.pi) / np.linalg.norm(seg.pf - seg.pi)
    
    # ahora calculo la distancia
    d = np.linalg.norm(np.array([x, y]) - pc)
    
    # Se evita la singularidad
    if d < 5:
        d = 5
    
    return d
