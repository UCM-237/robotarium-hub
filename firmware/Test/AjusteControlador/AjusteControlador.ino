/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  Robot.ino
 * RUTA:     /firmware/Robot.ino
 * REPO:     https://github.com/UCM-237/robotarium-hub
 * ----------------------------------------------------------------------------
*/


#include "common.h"
#include "controler.h"
#include "robot.h"
// --- CONFIGURACIÓN DEL TEST ---
double VELOCIDAD_OBJETIVO = 0.01; // rad/s (ajusta según necesites)
bool TEST_RUEDA_DERECHA = true;   // true para derecha, false para izquierda
// ------------------------------


robot miRobot;
controler PID_Rueda;


void setup() {
  Serial.begin(115200);
  miRobot.pinSetup();
  miRobot.motorSetup();
  double A=1.0;
  double B= 100.0;
  // Paso 1: Objetivo realista (aprox 2 vueltas por segundo)
  VELOCIDAD_OBJETIVO = 15.0; 
  PID_Rueda.setSetPoint(VELOCIDAD_OBJETIVO);
  PID_Rueda.setFeedForwardParam(10.0,0);

  // Paso 2: Solo proporcional (Kp). Ki y Kd a CERO.
  // Un Kp de 2.0 o 5.0 es un buen inicio para motores de bajo coste.
  PID_Rueda.setControlerParam(1.0, 0.0,0.0);
  // Configuración de interrupciones para encoders (necesario para calcular w real)
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isr_right, RISING);

  Serial.println("Objetivo(rad/s),Real(rad/s),PWM");
}

void loop() {
  static unsigned long lastMillis = 0;
    int pwm_output=0;
      
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 10) {
    lastMillis = millis();

    double w_real;

    // Calcular velocidad real (rad/s) basándonos en los encoders
    if (TEST_RUEDA_DERECHA) {
      // Cálculo simplificado basado en el tiempo entre pulsos (deltaTimeRight)
      // Nota: Asegúrate de que deltaTimeRight se actualice en la ISR
      w_real = (6.28318 / 20.0) / ((double)deltaTimeRight / 1000000.0); 
      //if (millis() - timeAfterRight > 100) w_real = 0; // Si no hay pulsos, velocidad 0

// 1. Calculamos la base necesaria para la velocidad objetivo (FF)
int base_pwm = PID_Rueda.feedForward(); 

// 2. Calculamos el ajuste fino (PID)
int ajuste_pid = PID_Rueda.pid(w_real);

// 3. Sumamos ambos. Si w_real < deseada, ajuste_pid será positivo y subirá de la base.
pwm_output = constrain(base_pwm + ajuste_pid, 110, 255);

miRobot.moveRightWheel(pwm_output, VELOCIDAD_OBJETIVO, false);    } 
    else {
      w_real = (6.28318 / 20.0) / ((double)deltaTimeLeft / 1000000.0);
     // if (millis() - timeAfterLeft > 100) w_real = 0;

      pwm_output = PID_Rueda.pid(w_real);
      miRobot.moveLeftWheel(pwm_output, VELOCIDAD_OBJETIVO, false);
    }

    // Formato para el Serial Plotter de Arduino
    Serial.print(VELOCIDAD_OBJETIVO);
    Serial.print(",");
    Serial.print(w_real);
    Serial.print(",");
    Serial.println(pwm_output); // O el PWM si quieres verlo
  }
}

// --- ISR Necesarias (Copiadas de tu lógica en Robot.ino) ---
void isr_right() {
unsigned long ahora = micros();
  // Bajamos el debounce a 1000 microsegundos (1ms)
  if (ahora - timeBeforeDebounceRight > 1000) { 
    deltaTimeRight = ahora - startTimeRight;
    startTimeRight = ahora;
    timeBeforeDebounceRight = ahora;
    timeAfterRight = ahora; // Usado para detectar si el robot se detuvo
    encoder_countRight++;
  }
  }

void isr_left() {
  timeAfterLeft = micros();
  deltaTimeLeft = timeAfterLeft - startTimeLeft;
  startTimeLeft = timeAfterLeft;
  encoder_countLeft++;
}
