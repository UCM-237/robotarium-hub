import math 

class orientationControl:
    """
    Controlador PID para orientar el robot.
    Mantiene el robot en el rumbo deseado ajustando la velocidad angular.
    """
    
    def __init__(self, sample_time, kp=2, ki=0.9, kd=0.4, windup_limit=14, derivative_filter=False, filter_coeff=0.5): 
        """
        Inicializa el controlador PID
        
        Args:
            sample_time: Período de muestreo en milisegundos
            kp: Ganancia proporcional (amplifica el error actual)
            ki: Ganancia integral (amplifica el error acumulado)
            kd: Ganancia derivativa (amplifica la velocidad de cambio del error)
            windup_limit: Límite de anti-windup para el término integral
            derivative_filter: Si aplicar filtro al término derivativo
            filter_coeff: Coeficiente del filtro derivativo
        """
        self.cumError = 0  # Error acumulado
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previousError = 0  # Error del paso anterior
        self.windup_limit = windup_limit
        self.sample_time = sample_time / 1000.0  # Convierte a segundos
        self.derivative_filter = derivative_filter
        self.filter_coeff = filter_coeff
        self.minimum_error = 0.628  # Dos pasos discretos de las ruedas
        self.previousDerivative = 0  # Derivada del paso anterior
        
    def PID(self, heading, newHeading):
        """
        Calcula la velocidad angular necesaria para girar hacia el rumbo deseado
        usando control PID
        
        Args:
            heading: Rumbo actual del robot (radianes)
            newHeading: Rumbo deseado (radianes)
            
        Devuelve:
            float: Velocidad angular de las ruedas
        """
        w = 0
        # Calcula el error de ángulo
        angleError = newHeading - heading
        
        # Normaliza el error al rango [-π, π]
        if angleError > math.pi:
            angleError = angleError - 2*math.pi
        elif angleError < (-math.pi):
            angleError = angleError + 2*math.pi
        
        # Suma el error actual al error acumulado
        self.cumError = self.cumError + angleError
        
        # Anti-windup: resetea el error integral si cambia de signo
        # (evita que se acumule en la dirección opuesta)
        if angleError < 0 and self.cumError > 0:
            self.cumError = 0   
        elif(angleError > 0 and self.cumError < 0):
            self.cumError = 0
        
        # Limita el error integral para evitar saturación
        if(self.cumError > 0):
            if(self.cumError > self.windup_limit):
                self.cumError = self.windup_limit 
        elif(self.cumError < 0):
            if(self.cumError < -self.windup_limit):
                self.cumError = -self.windup_limit
        
        # Calcula el término derivativo (velocidad de cambio del error)
        derivative = (angleError - self.previousError) / self.sample_time
        
        # Aplica filtro a la derivada si está configurado
        if self.derivative_filter:
            derivative = self.filter_coeff * derivative + (1 - self.filter_coeff) * self.previousDerivative
            self.previousDerivative = derivative
        
        # Solo aplica control si el error es significativo
        if(angleError > self.minimum_error or angleError < -self.minimum_error):
            # Fórmula PID: w = Kp*error + Ki*integral + Kd*derivada
            w = (self.kp*angleError) + self.ki*self.cumError*self.sample_time + self.kd*derivative
            print("w:", w)
        else:
            # Error muy pequeño, no necesita corrección
            w = 0
        
        # Guarda el error actual para el próximo cálculo
        self.previousError = angleError
        return w
            