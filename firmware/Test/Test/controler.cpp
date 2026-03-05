/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  controller.h / controller.cpp
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Implementación de los controladores de velocidad. Contiene la lógica del
 * algoritmo PID (Proporcional-Integral-Derivativo) y el modelo FeedForward
 * para la compensación de fricción y respuesta dinámica de los motores.
 * ----------------------------------------------------------------------------
 */
#include "controler.h"

// Constructor: Inicializa el límite de la suma del error integral para evitar saturación
controler::controler() {
    maxIntegralError = 14; 
}

controler::~controler() {}

// Configura las constantes del algoritmo PID
void controler::setControlerParam(double kp, double ki, double kd) {
    this->kp = kp; // Ganancia Proporcional: reacciona al error actual
    this->ki = ki; // Ganancia Integral: corrige errores acumulados en el tiempo
    this->kd = kd; // Ganancia Derivativa: reacciona a la velocidad del cambio (amortiguación)
}

// Algoritmo de Control PID
int controler::pid(double w) {   
    int output = 0;
    this->currentTime = millis();
    this->elapsedTime = (double)(0.01); // Tiempo de muestreo fijo (10ms)
    
    // Cálculo del error: Diferencia entre consigna (setPoint) y velocidad real (w)
    this->error = this->setPoint - w;
    double aux = abs(this->error);

    // Zona muerta: Si el error es menor a 0.3 rad/s, no aplicamos corrección para evitar oscilaciones
    if(aux >= 0.3) {
        // Acumulamos el error para la parte Integral
        this->cumError += this->error * this->elapsedTime;
        
        // Reinicio de Integral en cambio de signo (evita sobreimpulsos bruscos al cambiar de sentido)
        if((this->lastError > 0 && this->error < 0) || (this->lastError < 0 && this->error > 0)) {
            this->cumError = this->error * this->elapsedTime;
        }

        // Anti-windup: Limitamos el error acumulado para que el motor no se quede "atascado" al máximo
        constrain(this->cumError, -maxIntegralError, maxIntegralError);
        
        // Cálculo de la parte Derivativa (pendiente del error)
        this->rateError = (this->error - this->lastError) / this->elapsedTime;
        
        // Cálculo final de la salida: P + I (la parte D está comentada en el original para estabilidad)
        output = (int)(round(this->kp * this->error + this->ki * this->cumError)); 
        this->lastError = this->error;
    }
    
    this->previousTime = this->currentTime;
    return output;
}

// Configura los coeficientes de la recta de FeedForward (PWM = A*w + B)
void controler::setFeedForwardParam(double A, double B) {
    this->feedForwardParam_A = A;
    this->feedForwardParam_B = B;
}

// Predicción de potencia: Da un valor de PWM base según la velocidad deseada
int controler::feedForward() {
    // Es una aproximación lineal basada en la caracterización previa de tus motores
    this->PWM = (this->feedForwardParam_A * this->setPoint) + this->feedForwardParam_B;
    return constrain(this->PWM, MINPWM, MAXPWM);
}

void controler::setSetPoint(double setPoint) { this->setPoint = setPoint; }
double controler::getSetPoint() { return this->setPoint; }
