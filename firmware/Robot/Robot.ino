<<<<<<< HEAD
/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  Robot.ino
 * RUTA:     /firmware/Robot.ino
 * REPO:     https://github.com/UCM-237/robotarium-hub
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Punto de entrada principal (Entry Point). Gestiona el ciclo de vida del robot,
 * el bucle de control de tiempo real (100Hz) y el despacho de operaciones
 * recibidas por Serial1 desde la Raspberry Pi.
 * ----------------------------------------------------------------------------
 */
 
// Inclusión de librerías y archivos de cabecera del proyecto
#include "common.h"      // Estructuras de datos, uniones y variables de encoders
#include "controler.h"   // Clase para el control PID y FeedForward de los motores
#include "robot.h"       // Definiciones físicas y configuración de pines del robot
#include <math.h>
#include <SimpleKalmanFilter.h>

#ifdef ARDUINO_TYPE_MKR
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoJson.h>
#endif

#include "arduino_secrets.h" // Credenciales de red (SSID y Password)


// Configuración de macros para depuración por el Monitor Serie
#define DEBUG_ENABLED  
=======

//robot2
#include "common.h"
#include "controler.h"
#include "robot.h"
#include <math.h>
#include <SimpleKalmanFilter.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
// Define a macro for debug printing
#define DEBUG_ENABLED  // Comment out this line to disable debug prints

>>>>>>> 9fcf88d (Refactorización)
#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
<<<<<<< HEAD
#define DEBUG_PRINT(...)   
#define DEBUG_PRINTLN(...) 
#endif

#ifdef ARDUINO_TYPE_MKR
// Variables de conectividad WiFi
char ssid[] = SECRET_SSID;        
char pass[] = SECRET_PASS;    
int status = WL_IDLE_STATUS;     
#endif
using namespace std;
unsigned char packetBuffer[256]; // Buffer para almacenar paquetes recibidos por Serial

/// --- FILTROS Y ESTABILIZACIÓN ---
// Filtros de media móvil para suavizar las lecturas de velocidad de los encoders (ventana de 10 muestras)
MeanFilter<long> meanFilterRight(10);
MeanFilter<long> meanFilterLeft(10);
const double MAX_OPTIMAL_VEL=20; // Límite de seguridad en rad/s

// --- VARIABLES DE OPERACIÓN Y COMUNICACIÓN ---
struct appdata operation_send;    // Estructura para enviar datos a la Raspberry Pi
struct appdata *server_operation; // Puntero para interpretar los datos recibidos como estructura appdata

// Variables de tiempo para el bucle de control (Sampling Time)

bool control = false;   // Indica si el bucle PID debe estar activo
bool serialCom = false; // Indica si se ha recibido un comando completo por Serial

// --- PROTOTIPOS DE FUNCIONES ---
// Gestión de operaciones (Comandos de la Raspberry Pi)
void do_operation(int operation);
void op_saludo();      // Handshake inicial
void op_message();     // Procesamiento de mensajes genéricos
void op_moveRobot();   // Control cinemático diferencial (v, w)
void op_StopRobot();   // Parada de emergencia
void op_telemetry();   // Envío de estado de sensores y encoders
void op_turn_robot();  // Giro sobre el propio eje
void op_error();       // Respuesta ante comando desconocido

// Control de bajo nivel e interrupciones
void isrRight();       // Interrupción Rueda Derecha: cálculo de tiempos entre pulsos
void isrLeft();        // Interrupción Rueda Izquierda
void send(int operation, byte *data); // Envío de paquetes estructurados por Serial
void connect();        // Gestión de la conexión WiFi/MQTT

// --- VARIABLES GLOBALES DE ESTADO ---
controler wheelControlerRight; // Controlador PID para la rueda derecha
controler wheelControlerLeft;  // Controlador PID para la rueda izquierda
robot robot;                   // Objeto que representa el hardware físico del robot


int option;
int led = 13;          // LED integrado para señalización visual de estado


// Variables de temporización
unsigned long previous_timer;
unsigned long timer = 10000;
unsigned long currentTime, timeAfter;


// --- VARIABLES COMUNES DE CONTROL Y ESTADO ---

// Almacena el código de la operación que se está procesando actualmente por puerto serie
int serialOperation = 0;

// Bandera (flag) para habilitar o deshabilitar el envío de telemetría hacia la Raspberry Pi
// Si es true, el Arduino enviará datos de vuelta por el puerto serie
static bool sendDataSerial = false;

// Puntero de memoria de tipo carácter sin signo
// Se utiliza para recorrer y leer el buffer de datos recibidos byte a byte de forma eficiente
unsigned char *read_ptr; 

// Tiempo de muestreo del bucle de control expresado en milisegundos (10ms = 100Hz)
// Es el intervalo exacto en el que se ejecutan los cálculos del PID y el FeedForward
const unsigned long SAMPLINGTIME = 10;

// Variables para almacenar la velocidad angular medida (en radianes por segundo)
// wLeft: velocidad de la rueda izquierda | wRight: velocidad de la rueda derecha
// Estos valores provienen directamente del cálculo basado en los pulsos de los encoders
double wLeft, wRight;

// Bandera de inicio o "número mágico" para la validación de paquetes.
// Se usa para asegurar que el primer byte que recibe el Arduino es realmente
// el comienzo de un mensaje válido y no ruido o datos corruptos.
const int INIT_FLAG = 112;

#ifdef ARDUINO_TYPE_MKR
// Objeto que gestiona la conexión TCP/IP a través del chip WiFi del Arduino (MKR o Nano)
// Es el "túnel" de datos básico para cualquier comunicación por red
WiFiClient wifi;

// Cliente MQTT que utiliza el objeto 'wifi' para enviar y recibir mensajes
// MQTT es un protocolo de mensajería ligero basado en publicación/suscripción, ideal para IoT
MqttClient mqttClient(wifi);

// Dirección IP del servidor central (Broker MQTT)
// En este caso, apunta a un dispositivo en tu red local (probablemente la Raspberry Pi o un PC)
const char broker[] = "192.168.10.1";

// Puerto estándar para el protocolo MQTT (sin cifrado)
int mqttPort = 1883;

// Nombre identificador del robot ante el servidor MQTT
// Es fundamental para que el servidor sepa qué robot de los 10 está enviando datos
const char device[] = "arduinoClient";

// Para decodificar el JSON
StaticJsonDocument<200> doc;
#endif

/**
 * Configuración inicial del sistema.
 * Se ejecuta una sola vez al encender el robot o presionar reset.
 */
void setup() {
    // Inicializa la configuración de pines según el hardware (MKR/Nano y tipo de puente en H)
    robot.pinSetup();
    
    // Configura los pines de control de los motores (IN1, IN2, ENA, etc.) como salidas
    robot.motorSetup();
     
    // Configura los pines de los encoders como entrada con resistencia de pull-up interna
    // El pull-up asegura que el pin no quede "flotando" y evita lecturas erróneas de ruido
    pinMode(robot.getPinRightEncoder(), INPUT_PULLUP);
    pinMode(robot.getPinLeftEncoder(), INPUT_PULLUP);
    
    // Vincula los pines de los encoders a funciones de interrupción (ISR)
    // Se activan en el flanco de subida (RISING), permitiendo contar pulsos en tiempo real sin bloquear el loop
    attachInterrupt(digitalPinToInterrupt(robot.getPinLeftEncoder()), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(robot.getPinRightEncoder()), isrRight, RISING);
    
    // Inicializa la Unidad de Medición Inercial (IMU) interna para leer aceleración y rotación
    // Si la IMU no responde, el programa se detiene por seguridad para evitar errores de navegación
    /*if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1); // Bucle infinito de seguridad
    }*/

    // Asegura que el robot comience totalmente estático frenando ambos motores eléctricamente
    robot.fullStop();
  
    // Configuración del controlador de la rueda derecha:
    // setControlerParam: Ajusta las constantes PID (Kp=0.15, Ki=0.01, Kd=0.00)
    // setFeedForwardParam: Ajusta la compensación directa (Pendiente=0.0895, Offset=-5.424)
    wheelControlerRight.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerRight.setFeedForwardParam(0.0895, -5.424);

    // Configuración del controlador de la rueda izquierda:
    // Los parámetros varían ligeramente para compensar diferencias mecánicas entre motores
    wheelControlerLeft.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerLeft.setFeedForwardParam(0.0772, -3.173);

    // Inicializa la comunicación Serie 1 (pines físicos) con la Raspberry Pi a 9600 baudios
    Serial1.begin(9600);

    // Inicializa la comunicación Serie por USB para monitorización y depuración en el PC
    Serial.begin(9600); 

    #ifdef ARDUINO_TYPE_MKR
    WiFi.begin(ssid,pass);
    while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(500);
    }
    Serial.println(".");
    Serial.println("Conectado a Wifi");
    mqttClient.onMessage(onMqttMessage);
    Serial.println("Conectando al broker Mqtt");
     //Llamada para establecer conexión WiFi/MQTT (opcional)
    if(!mqttClient.connect(broker,mqttPort)){
      Serial.print("Conexion fallida");
      Serial.println(mqttClient.connectError());
      while(1);
    }
    const char topic[]="#";
    
    mqttClient.subscribe(topic);
    Serial.print("Suscrito al tema: ");
    Serial.println(topic);
    #endif
    // Mensaje de confirmación si la depuración está activa
    DEBUG_PRINTLN("setup ok"); 
}


void loop() {
 // 1. GESTIÓN DE TIEMPO Y COMUNICACIONES
  currentTime = millis(); // Registra el tiempo actual para el control de frecuencia
  serialEvent();          // Verifica si han llegado bytes por el puerto serie (Raspberry Pi)
  
  // 2. PROCESAMIENTO DE COMANDOS SERIALES
  if (serialCom) {
    // Depuración: imprime la cabecera y el código de operación recibidos
=======
#define DEBUG_PRINT(...)   // Debug print is empty when DEBUG_ENABLED is not defined
#define DEBUG_PRINTLN(...) // Debug println is empty when DEBUG_ENABLED is not defined
#endif
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status



using namespace std;
unsigned char packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------

MeanFilter<long> meanFilterRight(10);
MeanFilter<long> meanFilterLeft(10);
const double MAX_OPTIMAL_VEL=20;//rad/S



//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
//TODO: DO a reuqest manager for the opeartions and handle serial and mqtt communication
void do_operation(int operation);
void op_saludo();
void op_message();
void op_moveRobot();
void op_StopRobot();
void op_telemtry();
void op_turn_robot();
void op_error();//unrecognized operation
void isrRight();
void isrLeft();
void send(int operation, byte *data);
void connect();



int option;
int led = 13;
//Time variables
unsigned long previous_timer;
unsigned long timer=10000;
/*define controler*/
controler wheelControlerRight;
controler wheelControlerLeft;
/*define robot*/
robot robot;
WiFiClient wifi;
MqttClient mqttClient(wifi);
const char broker[] ="192.168.1.109";
int mqttPort = 1883;
const char device[] = "arduinoClient";

void setup() {
    robot.pinSetup();
    robot.motorSetup();
    // Interrupciones para contar pulsos de encoder
    pinMode(encoderRight, INPUT_PULLUP);
    pinMode(encoderLeft, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderLeft), isrLeft, RISING);//prepara la entrada del encoder como interrupcion
    attachInterrupt(digitalPinToInterrupt(encoderRight), isrRight, RISING);
    // Se prepara la IMU para poder ser leida
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    // Se empieza con los motores parados
    robot.fullStop();
  
    wheelControlerRight.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerRight.setFeedForwardParam(0.0895,-5.424);
    wheelControlerLeft.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerLeft.setFeedForwardParam(0.0772,-3.173);
    // Comunicacion por puerto serie
    Serial1.begin(9600);//for debuggin
    Serial.begin(9600);

    //connect();
   DEBUG_PRINTLN("setup ok");
}
//define common variables
//TODO: Refactor
int serialOperation = 0;
static bool sendDataSerial = false;
bool serialCom = false;
bool control = true;
unsigned char *read_ptr; 
unsigned long currentTime, timeAfter = 0;
const unsigned long SAMPLINGTIME= 10;//ms
double wLeft,wRight; // measured angular velocity

const int INIT_FLAG = 112;

void loop() {
  currentTime = millis();
  serialEvent();
  if (serialCom) 
  {
>>>>>>> 9fcf88d (Refactorización)
    DEBUG_PRINT("operationFlag: \t");
    DEBUG_PRINTLN(server_operation->InitFlag);
    DEBUG_PRINT("operation: \t");
    DEBUG_PRINTLN(server_operation->op);
<<<<<<< HEAD

    // Validación: Solo procesa si la cabecera coincide con INIT_FLAG (112)
    if (server_operation->InitFlag == INIT_FLAG) {
      DEBUG_PRINT("operationFlag: \t");
      DEBUG_PRINTLN(server_operation->InitFlag);
      DEBUG_PRINT("operation: \t");
      DEBUG_PRINTLN(server_operation->op);

      // Ejecuta la función correspondiente según el código de operación (OP_MOVE, OP_STOP, etc.)
      do_operation((operation_t)server_operation->op);
    }
    serialCom = false; // Reinicia la bandera tras procesar el paquete
  }
  
  // Mantiene viva la conexión MQTT
  #ifdef ARDUINO_TYPE_MKR
   //Call poll() regularly to allow the library to send MQTT keep alive which

  // avoids being disconnected by the broker
   mqttClient.poll();
  #endif
  
  // 3. PREPARACIÓN DE DATOS DE SENSORES
  int auxPWMD = 0, auxPWMI = 0; // Variables auxiliares para evitar enviar PWM repetido
  double fD, fI;                // Frecuencia de los encoders (pulsos por segundo)
  
  timeStopD = timeStopI = millis();
  
  // Cálculo del tiempo transcurrido desde el último pulso del encoder (Detección de parada)
  // Si este tiempo es muy alto, asumimos que la rueda no se mueve
  deltaTimeStopD = timeStopD - timeAfterDebounceRight;
  deltaTimeStopI = timeStopI - timeAfterDebounceLeft;
  
  // Añade los intervalos de tiempo entre pulsos a los filtros de media móvil
  meanFilterRight.AddValue(deltaTimeRight);
  meanFilterLeft.AddValue(deltaTimeLeft);

  // 4. BUCLE DE CONTROL DE TIEMPO REAL (Ejecutado cada SAMPLINGTIME, ej: 10ms)
  if(currentTime - timeAfter >= SAMPLINGTIME) 
  {
    // Cálculo de frecuencia (f = 1/T):
    // Si han pasado más de 100ms sin pulsos (deltaTimeStop >= 100), la frecuencia es 0.
    // Si no, se calcula: 1 / (tiempo_medio_entre_pulsos * resolución_encoder)
    fI = deltaTimeStopI >= 100 ? 0 : (double)1 / (meanFilterLeft.GetFiltered() * MAX_ENCODER_STEPS) * 1000;
    fD = deltaTimeStopD >= 100 ? 0 : (double)1 / (meanFilterRight.GetFiltered() * MAX_ENCODER_STEPS) * 1000;

    // FILTRO DE SEGURIDAD: Convierte frecuencia (Hz) a velocidad angular (rad/s)
    // Solo actualiza w si el valor es físicamente posible según el límite de seguridad
=======
    if (server_operation->InitFlag == INIT_FLAG)
    {
     DEBUG_PRINT("operationFlag: \t");
     DEBUG_PRINTLN(server_operation->InitFlag);
     DEBUG_PRINT("operation: \t");
     DEBUG_PRINTLN(server_operation->op);

      do_operation((operation_t)server_operation->op);
    }
    serialCom = false;
  }
  
  // call poll() regularly to allow the library to send MQTT keep alive which

  // avoids being disconnected by the broker

  //mqttClient.poll();
  int auxPWMD = 0, auxPWMI = 0;
  double fD, fI;
  timeStopD = timeStopI = millis();
  //condition for know when the wheel is stoped
  //se define un contador de tiempo para comprobar que las reudas estan paradas
  deltaTimeStopD = timeStopD - timeAfterDebounceRight;
  deltaTimeStopI = timeStopI - timeAfterDebounceLeft;
  meanFilterRight.AddValue(deltaTimeRight);
  meanFilterLeft.AddValue(deltaTimeLeft);
  

  if(currentTime - timeAfter >= SAMPLINGTIME) 
  {
    fI = deltaTimeStopI >= 100 ? 0 : (double)1 / (meanFilterLeft.GetFiltered() * MAX_ENCODER_STEPS) * 1000;
    fD = deltaTimeStopD >= 100 ? 0 : (double)1 / (meanFilterRight.GetFiltered() * MAX_ENCODER_STEPS) *1000;
    // DEBUG_PRINT("fI:");
    // DEBUG_PRINT(fI);
    // DEBUG_PRINT(" fD:");
    // DEBUG_PRINTLN(fD);
    //condicion para que no supere linealidad y se sature.
    //es un filtro para que no de valores ridiculos
>>>>>>> 9fcf88d (Refactorización)
    if(fD < double(MAX_OPTIMAL_VEL/2.0/M_PI)) 
    { 
      wRight = 2*M_PI*fD; 
    }
    if(fI < double(MAX_OPTIMAL_VEL/2.0/M_PI)) 
    { 
      wLeft = 2*M_PI*fI; 
    }
<<<<<<< HEAD
    
    // 5. ALGORITMO DE CONTROL PID
    if(control)
    { 
      // Control rueda Derecha: Solo actúa si hay movimiento y una consigna (Setpoint) distinta de cero
      if(wRight !=0.0 && wheelControlerRight.getSetPoint() !=0.0) {   
        int controllerValue =  wheelControlerRight.pid(wRight); // Obtiene corrección del PID
        // Ajusta el PWM acumulado y lo restringe al rango permitido (100-255)
        PWM_Right = constrain(PWM_Right + controllerValue, MINPWM, MAXPWM);
      }
      
      // Control rueda Izquierda
      if(wLeft !=0.0 && wheelControlerLeft.getSetPoint() !=0.0) {
        int controllerValue = wheelControlerLeft.pid(wLeft);
        PWM_Left = constrain(PWM_Left + controllerValue , MINPWM, MAXPWM);
      }
      
      // Impresión de depuración de velocidades angulares si hay movimiento
      if(wRight > 0){
        DEBUG_PRINT("wRight:");
        DEBUG_PRINT(wRight);
        DEBUG_PRINT(" wLeft:");
        DEBUG_PRINTLN(wLeft);
        
        Serial.print("wRight:");
        Serial.print(wRight);
        Serial.print(" wLeft:");
        Serial.println(wLeft);
      }
    }
    
   // 6. ACTUALIZACIÓN DE MOTORES (Escritura en Hardware)
    // Solo envía la instrucción al puente en H si el valor de PWM ha cambiado
    if(auxPWMI != PWM_Left){
      robot.moveLeftWheel(PWM_Left, wheelControlerLeft.getSetPoint(), backI);
      auxPWMI = PWM_Left; // Actualiza el valor auxiliar para la siguiente comparación
    }
    if(auxPWMD != PWM_Right){
=======
    //fin filtro
    if(control)
    { 
      
      if(wRight !=0.0 && wheelControlerRight.getSetPoint() !=0.0) {   
        int controllerValue =  wheelControlerRight.pid(wRight);
        PWM_Right = constrain(PWM_Right +controllerValue, MINPWM, MAXPWM);
      }
      if(wLeft !=0.0 && wheelControlerLeft.getSetPoint() !=0.0) {
        int controllerValue = wheelControlerLeft.pid(wLeft);
        PWM_Left = constrain(PWM_Left +controllerValue , MINPWM, MAXPWM);
      }
      // DEBUG_PRINT("PWM_Left:");
      // DEBUG_PRINT(PWM_Left);
      // DEBUG_PRINT(" PWM_Right:");
      // DEBUG_PRINTLN(PWM_Right);
      if(wRight >0){
      DEBUG_PRINT("wRight:");
      DEBUG_PRINT(wRight);
      DEBUG_PRINT(" wLeft:");
      DEBUG_PRINTLN(wLeft);
      
      Serial.print("wRight:");
      Serial.print(wRight);
      Serial.print(" wLeft:");
      Serial.println(wLeft);
      }
    }
   
    //avoid send the same instruction
    if(auxPWMI != PWM_Left){
      // moveWheel(PWM_Left, setpointWLeft, pinMotorI, backI);
      robot.moveLeftWheel(PWM_Left, wheelControlerLeft.getSetPoint(), backI);
      auxPWMI = PWM_Left;
    }
    if(auxPWMD != PWM_Right){
      //moveWheel(PWM_Right, setpointWRight, pinMotorD, backD);
>>>>>>> 9fcf88d (Refactorización)
      robot.moveRightWheel(PWM_Right, wheelControlerRight.getSetPoint(), backD);
      auxPWMD = PWM_Right;
    }

<<<<<<< HEAD
    // 7. TELEMETRÍA
    // Si la Raspberry Pi solicitó datos (sendDataSerial), envía el estado de los sensores
=======
    
>>>>>>> 9fcf88d (Refactorización)
    if(sendDataSerial)
    {
      op_telemtry();
    }
<<<<<<< HEAD
    
    // Actualiza la marca de tiempo para el próximo ciclo de muestreo
=======
>>>>>>> 9fcf88d (Refactorización)
    timeAfter = currentTime; 
  }

  
}

<<<<<<< HEAD
/**
 * Gestor de comandos (Dispatcher).
 * Recibe un enumerado de tipo operation_t y ejecuta la acción correspondiente.
 * Esta función conecta los mensajes de alto nivel con las acciones físicas del robot.
 */
void do_operation(operation_t operation) {
  switch (operation) {
    
    // Caso 1: Saludo inicial o Handshake.
    // Se usa para verificar que la Raspberry Pi y el Arduino están sincronizados.
    case OP_HELLO:
      op_saludo();
      break;

    // Caso 2: Movimiento coordinado (Cinemática diferencial).
    // Suele recibir velocidad lineal (v) y angular (w) para calcular ambas ruedas.
    case OP_MOVE_ROBOT:
      op_moveRobot();
      break;

    // Caso 3: Parada de emergencia.
    // Desactiva el control y frena los motores inmediatamente.
    case OP_STOP_ROBOT:
      op_StopRobot();
      break;

    // Caso 4: Activación de telemetría.
    // Cambia el flag global a 'true' para que el loop() empiece a enviar datos por Serial.
    case OP_TELEMETRY:
      sendDataSerial = true;
      break;

    // Caso 5: Giro preciso.
    // Ejecuta una rutina para que el robot rote sobre su propio eje central.
    case OP_TURN_ROBOT:
      op_turn_robot();
      break;

    // Caso 6: Silenciar telemetría.
    // Detiene el flujo de datos hacia la Raspberry Pi para liberar ancho de banda.
    case OP_SILENCE:
      op_silence(); // Nota: Asegúrate de que esta función ponga sendDataSerial = false;
      break;

    // Caso 8: Configuración dinámica del PID.
    // Permite ajustar Kp, Ki y Kd en tiempo real sin reiniciar el Arduino.
    case OP_CONF_PID:
      op_conf_pid();
      break;

    // Caso 9: Configuración dinámica del FeedForward.
    // Permite ajustar los parámetros A y B de la compensación de potencia.
    case OP_CONF_FF:
      op_conf_ff();
      break;

    // Caso 11: Control directo de ruedas.
    // Permite asignar velocidades (setpoints) individuales a cada rueda por separado.
    case OP_MOVE_WHEELS:
      op_moveWheels();
      break;

    // Caso por defecto: Error.
    // Si el código de operación no existe o está corrupto, notifica el fallo.
=======
void do_operation(operation_t operation) {
  switch (operation) {
    case OP_HELLO:
      op_saludo();
      break;
    case OP_MOVE_ROBOT:
      op_moveRobot();
      break;
    case OP_STOP_ROBOT:
      op_StopRobot();
      break;
    case OP_TELEMETRY:
      sendDataSerial = true;
      break;
    case OP_TURN_ROBOT:
      op_turn_robot();
      break;
    case OP_SILENCE:
      op_silence();
      break;
    case OP_CONF_PID:
      op_conf_pid();
      break;
    case OP_CONF_FF:
      op_conf_ff();
      break;
    case OP_MOVE_WHEELS:
      op_moveWheels();
      break;
>>>>>>> 9fcf88d (Refactorización)
    default:
      op_error();
      break;
  }
}

<<<<<<< HEAD
/**
 * Envía un paquete de datos estructurado a través del puerto Serie 1.
 * @param operation Código de la operación (ej. OP_TELEMETRY o OP_DONE).
 * @param data Puntero al array de bytes que contiene la información a enviar.
 */
void send(int operation, byte *data) {
  // Asigna el código de operación a la estructura de envío
  operation_send.op = operation;

  // Determina la longitud de los datos. 
  // NOTA: sizeof(data) aquí devolverá el tamaño del puntero (2 o 4 bytes), 
  // no el tamaño del array. Generalmente se prefiere pasar la longitud como parámetro.
  operation_send.len = sizeof(data);

  // Envía el código de operación (2 bytes / uint16_t)
  // Se realiza un cast a (char*) para que Serial1.write trate la dirección de memoria correctamente
  Serial1.write((char*)&operation_send.op, 2);

  // Envía la longitud de los datos (2 bytes / uint16_t)
  // Esto indica a la Raspberry Pi cuántos bytes debe esperar a continuación
  Serial1.write((char*)&operation_send.len, 2);

  // Envía el bloque de datos (la carga útil o payload)
  // Se envía desde la dirección de memoria apuntada por 'data'
  Serial1.write((char*)&data, operation_send.len);

  // Limpia el buffer de salida y espera a que se complete la transmisión física de los datos
  Serial1.flush();
}

/**
 * Operación de Saludo (Handshake).
 * Responde a la Raspberry Pi para confirmar que la comunicación está establecida.
 */
void op_saludo() {
  // Asigna el código de operación HELLO a la estructura de envío
  operation_send.op = OP_HELLO;

  // Define la longitud de los datos (en este caso el tamaño máximo del buffer de datos)
  operation_send.len = sizeof (operation_send.data); 
  
  // Envía por Serial1 el paquete completo: Carga útil + Cabecera (Header)
  // Se hace un cast a (char*) para enviar la estructura como un flujo de bytes puros
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  
  // Fuerza la salida de todos los bytes del buffer serie antes de continuar
  Serial1.flush();
}
// TODO: Completar
void op_message() { }
/**
 * Control directo de potencia en las ruedas.
 * Extrae los valores de PWM del paquete recibido y los aplica al hardware.
 */
void op_moveWheels()
{
  DEBUG_PRINTLN("move Wheels");

  // Conversión de bytes a enteros:
  // Los primeros 4 bytes [0-3] corresponden al PWM de la rueda derecha
  int PWMRight = bytesToLong(&server_operation->data[0]);
  // Los siguientes 4 bytes [4-7] corresponden al PWM de la rueda izquierda
  int PWMLeft = bytesToLong(&server_operation->data[4]);

  // Aplica el movimiento físico. 
  // Se pasa '1' como velocidad ficticia para que la función moveWheel no detecte parada.
  // 'false' indica que el movimiento por defecto es hacia adelante.
  robot.moveLeftWheel(PWMLeft, 1, false);
  robot.moveRightWheel(PWMRight, 1, false);

  // PRE-FILTRADO:
  // Se añaden 10 valores de golpe al filtro de media móvil para "limpiar" el histórico.
  // Esto ayuda a que el cálculo de velocidad angular (w) en el loop() no tenga 
  // picos bruscos justo después de recibir un comando de movimiento nuevo.
=======
void send(int operation, byte *data) {
  operation_send.op = operation;
  operation_send.len = sizeof(data);
  Serial1.write((char*)&operation_send.op, 1);
  Serial1.write((char*)&operation_send.len, 2);
  Serial1.write((char*)&data, operation_send.len);
  Serial1.flush();
}

void op_saludo() {
  operation_send.op = OP_HELLO;

  operation_send.len = sizeof (operation_send.data);  /* len */
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  Serial1.flush();
//  send(ID, OP_HELLO, )
}

void op_message() { }
void op_moveWheels()
{
  DEBUG_PRINTLN("move Wheels");
  int PWMRight = bytesToLong(&server_operation->data[0]);
  int PWMLeft = bytesToLong(&server_operation->data[4]);
  robot.moveLeftWheel(PWMLeft, 1, false);
  robot.moveRightWheel(PWMRight, 1, false);
>>>>>>> 9fcf88d (Refactorización)
  for(int i=0; i<10; i++)
  {
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
  }
}
<<<<<<< HEAD


/**
 * Operación de Movimiento Cinemático.
 * Traduce las velocidades deseadas (setpoints) enviadas por la Raspberry Pi
 * en señales de dirección y potencia (FeedForward) para los motores.
 */
void op_moveRobot() {
  DEBUG_PRINTLN("move");

  // 1. EXTRACCIÓN DE DATOS:
  // Se reconstruyen los valores 'double' (8 bytes cada uno) desde el buffer de datos.
  // Bytes 0-7: Velocidad deseada rueda derecha | Bytes 8-15: Velocidad deseada rueda izquierda.
  double setpointWRight = bytesToDouble(&server_operation->data[0]);
  double setpointWLeft = bytesToDouble(&server_operation->data[8]);

  DEBUG_PRINT("setpointWRight:");
  DEBUG_PRINT(setpointWRight);
  DEBUG_PRINT(" setpointWLeft:");
  DEBUG_PRINTLN(setpointWLeft);

  // 2. FILTRO DE ZONA MUERTA (Deadband):
  // Si la velocidad solicitada es muy baja (entre -1 y 1 rad/s), se fuerza a 0.
  // Esto evita que los motores "vibran" o zumben sin tener fuerza suficiente para moverse.
=======
void op_moveRobot() {
 DEBUG_PRINTLN("move");
  double setpointWRight = bytesToDouble(&server_operation->data[0]);
  double setpointWLeft = bytesToDouble(&server_operation->data[8]);
  DEBUG_PRINT("setpointWRight:");
  DEBUG_PRINT(setpointWRight);

  DEBUG_PRINT(" setpointWLeft:");
  DEBUG_PRINTLN(setpointWLeft);
>>>>>>> 9fcf88d (Refactorización)
  if(setpointWRight < 1 && setpointWRight > -1) {
    setpointWRight = 0;
  }
  if(setpointWLeft < 1 && setpointWLeft > -1) {
    setpointWLeft = 0;
  }
<<<<<<< HEAD

  // 3. LÓGICA DE DIRECCIÓN (Rueda Derecha):
  // Si el valor es negativo, el robot debe ir hacia atrás.
  if(setpointWRight < 0) {
    setpointWRight = setpointWRight * (-1); // Convertimos a valor absoluto para el controlador
    backD = true;                           // Activamos bandera de marcha atrás
  } else if(setpointWRight > 0) {
    backD = false;                          // Marcha hacia adelante
  }

  // LÓGICA DE DIRECCIÓN (Rueda Izquierda):
  if(setpointWLeft < 0) {
    setpointWLeft = setpointWLeft * (-1);
    backI = true;
  } else if(setpointWLeft > 0) {
    backI = false;
  }

  // 4. ACTUALIZACIÓN DEL CONTROLADOR:
  // Se informa a los objetos de control cuál es la nueva velocidad objetivo.
  wheelControlerLeft.setSetPoint(setpointWLeft);
  wheelControlerRight.setSetPoint(setpointWRight);

  // 5. CÁLCULO DE POTENCIA INICIAL (FeedForward):
  // El FeedForward estima el PWM necesario basándose en la velocidad deseada 
  // antes de que el PID empiece a corregir errores.
  PWM_Left = wheelControlerLeft.feedForward();
  PWM_Right = wheelControlerRight.feedForward();

=======
  if(setpointWRight < 0) {
    setpointWRight = setpointWRight*(-1);
    backD = true;
  } else if(setpointWRight > 0) {
    backD = false;
  }
  if(setpointWLeft < 0) {
    setpointWLeft = setpointWLeft*(-1);
    backI = true;
  }
  else if(setpointWLeft>0) {
    backI = false;
  }
  wheelControlerLeft.setSetPoint(setpointWLeft);
  wheelControlerRight.setSetPoint(setpointWRight);

  PWM_Left=wheelControlerLeft.feedForward();
  PWM_Right=wheelControlerRight.feedForward();
>>>>>>> 9fcf88d (Refactorización)
  DEBUG_PRINT("PWM_Left:");
  DEBUG_PRINT(PWM_Left);
  DEBUG_PRINT(" PWM_Right:");
  DEBUG_PRINTLN(PWM_Right);
<<<<<<< HEAD

  // 6. EJECUCIÓN FÍSICA:
  // Se envían las señales a los puentes en H a través de la clase robot.
  robot.moveLeftWheel(PWM_Left, setpointWLeft, backI);
  robot.moveRightWheel(PWM_Right, setpointWRight, backD);

  // 7. ESTABILIZACIÓN DE SENSORES:
  // Se fuerzan 10 lecturas iniciales en el filtro de media móvil para que 
  // el sistema de control de velocidad no herede datos de cuando el robot estaba en otro estado.
=======
  // feedForwardD();
  // feedForwardI();  
  // moveWheel(PWM_Left, setpointWLeft, pinMotorI, backI);
  // moveWheel(PWM_Right, setpointWRight, pinMotorD, backD);
  robot.moveLeftWheel(PWM_Left, setpointWLeft, backI);
  robot.moveRightWheel(PWM_Right, setpointWRight, backD);
  // robot.moveLeftWheel(PWM_Left, setpointWLeft, backI);
  // robot.moveRightWheel(PWM_Right, setpointWRight, backD);
  //take the mean of the last 5 values for measure the angular velocity 
  //of every wheel
>>>>>>> 9fcf88d (Refactorización)
  for(int i=0; i<10; i++)
  {
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
  }
}

<<<<<<< HEAD

/**
 * Operación de Parada del Robot.
 * Detiene el movimiento de forma controlada y segura, reseteando los controladores.
 */
void op_StopRobot() {
  // Notificación por el puerto de depuración (USB)
  DEBUG_PRINT("op stop:");
  DEBUG_PRINTLN(OP_STOP_ROBOT);

  // 1. RESET DE SETPOINTS:
  // Se establece la velocidad objetivo a 0.0 rad/s en ambos controladores.
  // Esto es vital para que, si el PID sigue activo, no intente compensar nada.
  wheelControlerLeft.setSetPoint(0.0);
  wheelControlerRight.setSetPoint(0.0);

  // 2. PARADA FÍSICA (Hardware):
  // Llama al método de la clase robot que gestiona el puente en H.
  // A diferencia de simplemente poner el PWM a 0, fullStop suele aplicar un "freno motor"
  // poniendo ambos pines del motor (IN1/IN2) en el mismo estado lógico (HIGH/HIGH).
  robot.fullStop();
}


/**
 * Operación de Telemetría.
 * Empaqueta y envía el estado actual del robot (velocidad real y PWM) 
 * hacia la Raspberry Pi a través del puerto Serie 1.
 */
void op_telemtry() {
  // Notificación de depuración para saber que se está enviando el reporte
  DEBUG_PRINT("op telemetry:");
  DEBUG_PRINTLN(OP_TELEMETRY);

  // 1. CONFIGURACIÓN DE LA CABECERA (HEADER)
  // Se prepara la estructura con los datos de identificación del paquete
  operation_send.InitFlag = (int)INIT_FLAG; // Bandera 112 para sincronización
  operation_send.id = robot.getRobotID();   // ID único del robot (ej. 5)
  operation_send.op = (int)OP_TELEMETRY;    // Código de la operación actual

  // 2. EMPAQUETADO DE DATOS (PAYLOAD):
  // Se convierten los valores numéricos a bytes para su transmisión binaria.
  
  // Bytes 0-7: Velocidad angular medida en la rueda izquierda (double, 8 bytes)
  doubleToBytes(wLeft, &operation_send.data[0]);
  
  // Bytes 8-15: Velocidad angular medida en la rueda derecha (double, 8 bytes)
  doubleToBytes(wRight, &operation_send.data[8]);
  
  // Bytes 16-19: Potencia PWM actual de la rueda izquierda (long, 4 bytes)
  longToBytes(PWM_Left, &operation_send.data[16]);
  
  // Bytes 20-23: Potencia PWM actual de la rueda derecha (long, 4 bytes)
  longToBytes(PWM_Right, &operation_send.data[20]);

  // Depuración por USB para monitorizar la velocidad medida en tiempo real
=======
void op_StopRobot() {
  DEBUG_PRINT("op stop:");
  DEBUG_PRINTLN(OP_STOP_ROBOT);
  wheelControlerLeft.setSetPoint(0.0);
  wheelControlerRight.setSetPoint(0.0);
  // fullStop(pinMotorI);
  // fullStop(pinMotorD);
  robot.fullStop();
}

void op_telemtry() {
  DEBUG_PRINT("op telemetry:");
  DEBUG_PRINTLN(OP_TELEMETRY);
  operation_send.InitFlag=(int)INIT_FLAG;
  operation_send.id=robot.getRobotID();
  operation_send.op =(int)OP_TELEMETRY;
  short int a=1;
  doubleToBytes(wLeft, &operation_send.data[0]);
  doubleToBytes(wRight, &operation_send.data[8]);
  longToBytes(PWM_Left, &operation_send.data[16]);
  longToBytes(PWM_Right, &operation_send.data[20]);
>>>>>>> 9fcf88d (Refactorización)
  DEBUG_PRINT("vel robot--->");
  DEBUG_PRINT(" wRight:");
  DEBUG_PRINT(wRight);
  DEBUG_PRINT(" wLeft:");
  DEBUG_PRINTLN(wLeft);
<<<<<<< HEAD

  // 3. CÁLCULO DE LONGITUD:
  // Se define el tamaño total de los datos: (2 doubles * 8 bytes) + (2 longs * 4 bytes) = 24 bytes
  operation_send.len = (int)sizeof(double)*2 + sizeof(int)*2; 

  DEBUG_PRINT("len: ");
  DEBUG_PRINTLN((int)operation_send.len);

  // 4. ENVÍO BINARIO POR SERIAL1:
  // Se envían campo por campo asegurando que cada uno ocupe 2 bytes en el flujo serie
  Serial1.write((char*)&operation_send.InitFlag, 2);
  Serial1.write((char*)&operation_send.id, 2);
  Serial1.write((char*)&operation_send.op, 2);
  Serial1.write((char*)&operation_send.len, 2);
  
  // Envía el bloque de datos de telemetría (los 24 bytes calculados)
  Serial1.write((char*)&operation_send.data, operation_send.len);

  // Asegura que no se pierda ningún bit en el buffer de salida
  Serial1.flush();
}

/**
 * Operación de Giro del Robot.
 * Realiza una rotación precisa sobre el eje central basándose en el conteo de encoders.
 * Calcula la distancia que debe recorrer cada rueda para cumplir el ángulo solicitado.
 */
=======
  operation_send.len = (int)sizeof(double)*2+sizeof(int)*2;  /* len */
  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();
  //send(ID, OP_TELEMETRY, &operation_send.data);
}
>>>>>>> 9fcf88d (Refactorización)
void op_turn_robot()
{
  DEBUG_PRINT("op turn:");
  DEBUG_PRINTLN(OP_TURN_ROBOT);
<<<<<<< HEAD
  
  bool turnRight;
  // Extrae el ángulo deseado (en grados) de los primeros 4 bytes del paquete
  int angle = bytesToLong(&server_operation->data[0]);
  
  DEBUG_PRINT("angle:");
  DEBUG_PRINTLN(angle);

  // 1. DETERMINACIÓN DEL SENTIDO DE GIRO:
  // Si el ángulo es negativo, giramos a la derecha; si es positivo, a la izquierda.
  if(angle < 0)
  {
    turnRight = true;
    angle = angle * (-1); // Trabajamos con el valor absoluto para los cálculos
=======
  bool turnRight;
  int angle = bytesToLong(&server_operation->data[0]);
  DEBUG_PRINT("angle:");
  DEBUG_PRINTLN(angle);
  if(angle<0)
  {
    turnRight = true;
    angle = angle*(-1);
>>>>>>> 9fcf88d (Refactorización)
  }
  else
  {
    turnRight = false;
  }
<<<<<<< HEAD

  // 2. CÁLCULOS GEOMÉTRICOS (Cinemática):
  // Convertimos el ángulo de grados a radianes
  double angleInRad = ((double)angle) * M_PI / 180.0;
  
  // Calculamos la distancia lineal (arco) que debe recorrer cada rueda.
  // s = θ * r, donde r es la mitad del diámetro del robot (distancia centro-rueda).
  double angleToTurn = angleInRad * (robot.getRobotDiameter()) / 2.0;

  // 3. REINICIO DE ODOMETRÍA:
  // Ponemos a cero los contadores de pulsos para empezar la medición desde cero
  encoder_countRight = 0;
  encoder_countLeft = 0;

  // 4. CÁLCULO DEL OBJETIVO DE PULSOS (Target):
  // Convertimos la distancia lineal 'angleToTurn' en número de pulsos del encoder.
  // pasos = (distancia / circunferencia_rueda) * pulsos_por_vuelta
  int targetEncoderCount = int(angleToTurn / (2 * M_PI * robot.getRobotWheelRadius()) * MAX_ENCODER_STEPS);

  // 5. BUCLE DE EJECUCIÓN (Bloqueante):
  // El robot se moverá hasta que ambas ruedas hayan alcanzado el número de pulsos objetivo.
  while (encoder_countRight < targetEncoderCount && encoder_countLeft < targetEncoderCount)
  {
    // Al ser booleano, lo correcto es evaluar true/false.
    if(turnRight) 
    {
      // Girar a la derecha: Rueda izquierda adelante, Rueda derecha atrás
=======
  //If angle is negative, turn right else turn left
  double angleInRad = ((double)angle)*M_PI/180;
  double angleToTurn = angleInRad*(robot.getRobotDiameter())/2;
  //Reset the encoder count
  encoder_countRight=0;
  encoder_countLeft=0;
  //Set the target encoder count
  int targetEncoderCount = int(angleToTurn/(2*M_PI*robot.getRobotWheelRadius())*MAX_ENCODER_STEPS);
  while (encoder_countRight < targetEncoderCount && encoder_countLeft < targetEncoderCount)
  {
    if(turnRight<0)
    {
>>>>>>> 9fcf88d (Refactorización)
      robot.moveLeftWheel(MINPWM, 1, false);
      robot.moveRightWheel(MINPWM, 1, true);
    }
    else
    {
<<<<<<< HEAD
      // Girar a la izquierda: Rueda izquierda atrás, Rueda derecha adelante
=======
>>>>>>> 9fcf88d (Refactorización)
      robot.moveLeftWheel(MINPWM, 1, true);
      robot.moveRightWheel(MINPWM, 1, false);
    }
  }
<<<<<<< HEAD

  // 6. FINALIZACIÓN:
  // Una vez alcanzado el ángulo, frenamos en seco
  robot.fullStop();
  
  // Enviamos confirmación a la Raspberry Pi de que la tarea ha terminado
  op_done();
}


/**
 * Operación de Silencio.
 * Desactiva el envío automático de telemetría para liberar el bus de datos.
 */
=======
  robot.fullStop();
  op_done();
}
>>>>>>> 9fcf88d (Refactorización)
inline void op_silence()
{
  DEBUG_PRINT("op silense:");
  DEBUG_PRINTLN(OP_SILENCE);
<<<<<<< HEAD

  // Desactiva la bandera que permite el envío de datos en el loop()
  sendDataSerial = false;

  // Confirma a la Raspberry Pi que la orden ha sido procesada
  op_done();
}

/**
 * Operación de Error.
 * Se ejecuta cuando el robot recibe un comando que no sabe procesar.
 * Envía un paquete de notificación de vuelta al servidor.
 */
=======
  sendDataSerial = false;
  op_done();
}

>>>>>>> 9fcf88d (Refactorización)
inline void op_error()
{
  DEBUG_PRINT("op error:");
  DEBUG_PRINTLN(OP_ERROR);
<<<<<<< HEAD

  // 1. PREPARACIÓN DEL PAQUETE DE ERROR
  operation_send.op = OP_ERROR;
  operation_send.InitFlag = INIT_FLAG;
  operation_send.id = (int)robot.getRobotID();
  operation_send.len = sizeof(operation_send.data); /* Tamaño del buffer de datos */

  // 2. ENVÍO BINARIO

  Serial1.write((char*)&operation_send.InitFlag, 2);
  Serial1.write((char*)&operation_send.id, 2);
  Serial1.write((char*)&operation_send.op, 2);
  Serial1.write((char*)&operation_send.len, 2);
  
  // Envía el bloque de datos (aunque en un error suelen ser ceros o estar vacíos)
  Serial1.write((char*)&operation_send.data, operation_send.len);
  
  // Limpia el buffer de salida
  Serial1.flush();
}
/**
 * Configuración dinámica de los parámetros PID.
 * Extrae las constantes Kp, Ki y Kd para ambas ruedas desde el buffer de datos
 * y las aplica a los controladores en tiempo real.
 */
=======
  operation_send.op = OP_ERROR;
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=(int)robot.getRobotID();
  operation_send.len = sizeof (operation_send.data);  /* len */

  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();
}
>>>>>>> 9fcf88d (Refactorización)
void op_conf_pid()
{
    DEBUG_PRINT("conf pid:");
    DEBUG_PRINTLN(OP_CONF_PID);
<<<<<<< HEAD

    // 1. EXTRACCIÓN DE CONSTANTES PARA LA RUEDA DERECHA:
    // Cada valor 'double' ocupa 8 bytes en el buffer.
    double kp_right = bytesToDouble(&server_operation->data[0]);  // Proporcional
    double ki_right = bytesToDouble(&server_operation->data[8]);  // Integral
    double kd_right = bytesToDouble(&server_operation->data[16]); // Derivativo

    // 2. EXTRACCIÓN DE CONSTANTES PARA LA RUEDA IZQUIERDA:
    // Se continúa desde el offset 24 para evitar solapamiento.
=======
    double kp_right = bytesToDouble(&server_operation->data[0]);
    double ki_right = bytesToDouble(&server_operation->data[8]);
    double kd_right = bytesToDouble(&server_operation->data[16]);

>>>>>>> 9fcf88d (Refactorización)
    double kp_left = bytesToDouble(&server_operation->data[24]);
    double ki_left = bytesToDouble(&server_operation->data[32]);
    double kd_left = bytesToDouble(&server_operation->data[40]);

<<<<<<< HEAD
    // 3. ACTUALIZACIÓN DE LOS CONTROLADORES:
    // Se pasan los nuevos valores a los objetos wheelControler.
    // Esto reinicia el comportamiento de cálculo en la siguiente iteración del loop().
    wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
    wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);

    // 4. CONFIRMACIÓN Y DEPURACIÓN:
    // Envía la señal 'op_done' a la Raspberry Pi para confirmar la recepción.
    op_done();

    // Imprime los valores recibidos por el monitor serie para verificación visual.
    DEBUG_PRINT(kp_right); DEBUG_PRINT(",");
    DEBUG_PRINT(ki_right); DEBUG_PRINT(",");
    DEBUG_PRINT(kd_right); DEBUG_PRINT(",");
    DEBUG_PRINT(kp_left);  DEBUG_PRINT(",");
    DEBUG_PRINT(ki_left);  DEBUG_PRINT(",");
    DEBUG_PRINT(kd_left);
}

/**
 * Configuración dinámica de parámetros FeedForward (FF).
 * Ajusta la pendiente (A) y el offset (B) del modelo lineal de cada motor.
 * Esto permite compensar el rozamiento y la respuesta eléctrica de los motores.
 */
=======
    wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
    wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);

    op_done();
    DEBUG_PRINT(kp_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(ki_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(kd_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(kp_left);
    DEBUG_PRINT(",");
    DEBUG_PRINT(ki_left);
    DEBUG_PRINT(",");
    DEBUG_PRINT(kd_left);

}

>>>>>>> 9fcf88d (Refactorización)
void op_conf_ff()
{
  DEBUG_PRINT("conf ff:");
  DEBUG_PRINTLN(OP_CONF_FF);
<<<<<<< HEAD

  // 1. EXTRACCIÓN DE PARÁMETROS PARA LA RUEDA DERECHA:
  // A (Pendiente): Relación entre velocidad angular y PWM.
  // B (Offset): PWM mínimo necesario para romper la inercia (fricción estática).
  double A_right = bytesToDouble(&server_operation->data[0]);
  double B_right = bytesToDouble(&server_operation->data[8]);

  // 2. EXTRACCIÓN DE PARÁMETROS PARA LA RUEDA IZQUIERDA:
  // Se extraen de los siguientes bloques de 8 bytes.
  double A_left = bytesToDouble(&server_operation->data[16]);
  double B_left = bytesToDouble(&server_operation->data[24]);

  // 3. ACTUALIZACIÓN DE LOS CONTROLADORES:
  // Aplica los nuevos modelos matemáticos a cada objeto de control.
  wheelControlerRight.setFeedForwardParam(A_right, B_right);
  wheelControlerLeft.setFeedForwardParam(A_left, B_left);

  // 4. DEPURACIÓN Y CONFIRMACIÓN:
  // Imprime los nuevos valores para verificar que la transmisión fue correcta.
  DEBUG_PRINT(A_right);
  DEBUG_PRINT(",");
  DEBUG_PRINT(B_right);
  DEBUG_PRINT(",");
  DEBUG_PRINT(A_left);
  DEBUG_PRINT(",");
  DEBUG_PRINTLN(B_left);

  // Informa a la Raspberry Pi que los parámetros han sido actualizados con éxito.
  op_done();
}
/**
 * Operación de Confirmación (ACK).
 * Notifica a la Raspberry Pi que la operación solicitada se ha completado.
 * Es vital para que el software de nivel superior sepa que puede enviar la siguiente orden.
 */
void op_done()
{
  // Depuración local por el puerto USB
  DEBUG_PRINT("op done:");
  DEBUG_PRINTLN(OP_DONE);

  // 1. PREPARACIÓN DEL PAQUETE DE RESPUESTA
  operation_send.op = OP_DONE;             // Código de operación: 106 (generalmente)
  operation_send.InitFlag = INIT_FLAG;      // Cabecera de sincronización: 112
  operation_send.id = (int)robot.getRobotID(); // Identificación de la unidad
  operation_send.len = sizeof(operation_send.data); /* Tamaño del payload */

  // 2. ENVÍO BINARIO POR SERIAL1 (Protocolo de 2 bytes)

  Serial1.write((char*)&operation_send.InitFlag, 2);
  Serial1.write((char*)&operation_send.id, 2);
  Serial1.write((char*)&operation_send.op, 2);
  Serial1.write((char*)&operation_send.len, 2);
  
  // Envía el bloque de datos (aunque en un 'done' suelen ser ceros)
  Serial1.write((char*)&operation_send.data, operation_send.len);
  
  // Limpia el buffer de salida para garantizar el envío inmediato
  Serial1.flush();
}


/**
 * ISR para la rueda derecha.
 * Se activa en cada flanco de subida del sensor del encoder.
 */
void isrRight() {
  // 1. GESTIÓN DE REBOTES (Debouncing)
  timeBeforeDebounceRight = millis(); // Captura el tiempo actual del pulso
  deltaDebounceRight = timeBeforeDebounceRight - timeAfterDebounceRight; // Tiempo desde el último pulso (aunque fuera ruido)

  // Solo procesamos el pulso si ha pasado suficiente tiempo (TIMEDEBOUNCE)
  // Esto filtra picos de voltaje o vibraciones mecánicas que darían velocidades falsas
  if(deltaDebounceRight > TIMEDEBOUNCE) {
    
    // 2. CONTEO DE ODOMETRÍA
    startTimeRight = millis(); // Marca el tiempo de inicio de este pulso "válido"
    encoder_countRight++;      // Incrementa el contador total de pasos
    
    // Si llegamos al número de pasos por vuelta, incrementamos el contador de vueltas completas
    if(encoder_countRight == MAX_ENCODER_STEPS) {
      wheelTurnCounterRight++;
    }

    // 3. CÁLCULO DE VELOCIDAD (Delta Time)
    // Calcula el tiempo exacto transcurrido entre este pulso y el anterior
    // Este valor es inversamente proporcional a la velocidad de la rueda
    deltaTimeRight = startTimeRight - timeAfterRight;
     
    // Guarda este instante para compararlo con el próximo pulso
    timeAfterRight = startTimeRight;
  }
  
  // Actualiza el registro de tiempo para el siguiente chequeo de rebote
  timeAfterDebounceRight = timeBeforeDebounceRight;   
}

/**
 * ISR para la rueda izquierda.
 * Realiza la misma lógica de filtrado y conteo para el motor izquierdo.
 */
void isrLeft() {
  timeBeforeDebounceLeft = millis();
  deltaDebounceLeft = timeBeforeDebounceLeft - timeAfterDebounceLeft;

  if(deltaDebounceLeft > TIMEDEBOUNCE) {
    startTimeLeft = millis();
    encoder_countLeft++; 

    if(encoder_countLeft == MAX_ENCODER_STEPS) {
      wheelTurnCounterLeft++;
    }

    // El deltaTimeLeft es lo que luego usa el filtro de media móvil en el loop()
=======
  double A_right = bytesToDouble(&server_operation->data[0]);
  double B_right = bytesToDouble(&server_operation->data[8]);

  double A_left = bytesToDouble(&server_operation->data[16]);
  double B_left = bytesToDouble(&server_operation->data[24]);

  wheelControlerRight.setFeedForwardParam(A_right, B_right);
  wheelControlerLeft.setFeedForwardParam(A_left, B_left);

  DEBUG_PRINT(A_right);
 DEBUG_PRINT(",");
  
  DEBUG_PRINT(B_right);
  DEBUG_PRINT(",");
  
  DEBUG_PRINT(A_left);
  DEBUG_PRINT(",");
  
  DEBUG_PRINTLN(B_left);
  op_done();
}
void op_done()
{
  DEBUG_PRINT("op done:");
  DEBUG_PRINTLN(OP_DONE);
  operation_send.op = OP_DONE;
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=(int)robot.getRobotID();
  operation_send.len = sizeof (operation_send.data);  /* len */

  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();

}
void isrRight() {
  timeBeforeDebounceRight = millis();//tiempo para evitar rebotes
  deltaDebounceRight = timeBeforeDebounceRight - timeAfterDebounceRight;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceRight > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeRight=millis();
    encoder_countRight++;
    if(encoder_countRight == MAX_ENCODER_STEPS)
    {
      wheelTurnCounterRight++;
    }
    deltaTimeRight = startTimeRight - timeAfterRight;
      
    timeAfterRight = startTimeRight;
  }
  timeAfterDebounceRight = timeBeforeDebounceRight;  
}

void isrLeft() {
  timeBeforeDebounceLeft = millis();//tiempo para evitar rebotes
  deltaDebounceLeft = timeBeforeDebounceLeft - timeAfterDebounceLeft;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceLeft > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeLeft = millis();
    encoder_countLeft++;//se cuenta los pasos de encoder
    if(encoder_countLeft == MAX_ENCODER_STEPS)
    {
      wheelTurnCounterLeft++;
    
    }
>>>>>>> 9fcf88d (Refactorización)
    deltaTimeLeft = startTimeLeft - timeAfterLeft;
    timeAfterLeft = startTimeLeft;
  }
  timeAfterDebounceLeft = timeBeforeDebounceLeft;
}


void serialEvent() {
<<<<<<< HEAD
  // Verifica si hay al menos un byte esperando en el buffer de entrada
  if(Serial1.available())
  {
    // Lee una ráfaga de bytes y los guarda en 'packetBuffer'
    // Se detiene si: encuentra un '\n', llena el buffer, o pasan 1000ms (timeout por defecto)
    int rlen = Serial1.readBytesUntil('\n', (char*)&packetBuffer, sizeof(packetBuffer));
    
    // Mapea la estructura 'server_operation' sobre el buffer de bytes recibidos
    // Esto permite acceder a packetBuffer[0] como server_operation->InitFlag, etc.
    server_operation = (struct appdata *)&packetBuffer;
    
    // Indica al loop() que hay un mensaje listo para ser procesado
    serialCom = true;

    // Imprime la cantidad de bytes leídos para depuración
=======
  // read_ptr = (unsigned char*)&packetBuffer;
  // while (Serial1.available()) {
  //   *(read_ptr++) = Serial1.read();
  //   server_operation = (struct appdata *)&packetBuffer;
  //   serialCom = true;
  // }
  if(Serial1.available())
  {
    int rlen = Serial1.readBytesUntil('\n', (char*)&packetBuffer, sizeof(packetBuffer));
    server_operation = (struct appdata *)&packetBuffer;
    serialCom = true;
>>>>>>> 9fcf88d (Refactorización)
    DEBUG_PRINTLN(rlen);
  }
}

<<<<<<< HEAD
/* TODO: Revisar esta version mejorada no bloqueante
 *  void serialEvent() {
  static int index = 0; // Mantiene la posición del buffer entre llamadas a la función

  // Mientras haya datos, los procesamos uno a uno sin detener el resto del programa
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    // 1. Sincronización: Si el buffer está vacío, el primer byte DEBE ser el INIT_FLAG (112)
    // Si no lo es, descartamos el byte y seguimos buscando el inicio real.
    if (index == 0 && c != (char)INIT_FLAG) {
      continue; 
    }

    // 2. Almacenamos el byte y avanzamos
    ((char*)&packetBuffer)[index++] = c;

    // 3. Verificación de final de paquete
    // Suponemos que un paquete completo tiene el tamaño de tu estructura appdata
    if (index >= sizeof(struct appdata)) {
      server_operation = (struct appdata *)&packetBuffer;
      
      // Solo aceptamos el paquete si el InitFlag es correcto
      if (server_operation->InitFlag == INIT_FLAG) {
        serialCom = true;
      }
      
      index = 0; // Reiniciamos para el próximo mensaje
    }
  }
}
 */

 

#ifdef ARDUINO_TYPE_MKR

void onMqttMessage(int messageSize){
  Serial.print("Mensaje recibido en el topic ");
  Serial.println(mqttClient.messageTopic());
  Serial.print(" Tamaño: ");
  Serial.print(messageSize);
  Serial.print(" bytes");
  String incoming = "";

    while(mqttClient.available()){
      incoming += (char)mqttClient.read();
    }
  DeserializationError error = deserializeJson(doc, incoming);
  float x = doc["x"], y = doc["y"], yaw = doc["yaw"];
  Serial.println();
  Serial.print("x=");
  Serial.print(x); 
  Serial.print(", y=");
  Serial.print(y); 
  Serial.print(", yaw=");
  Serial.println(yaw); 

  if (error) {
    Serial.print("Error: ");
    Serial.println(error.c_str());
    return;
  }
  Serial.println("--------------------------------------------------");
}
/**
 * Formatea un array de 6 bytes en una cadena hexadecimal separada por puntos.
 * @param mac Array de 6 bytes con la dirección física.
 */
void printMacAddress(byte mac[]) {
  // Recorre los 6 bytes de la MAC
  for (int i = 5; i >= 0; i--) {
    // Si el valor es menor a 16 (un solo dígito hex), añade un '0' a la izquierda
    // para mantener el formato de dos dígitos (ej. "0A" en lugar de "A")
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    
    // Añade el separador ':' entre los pares de bytes
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
/**
 * Resume y muestra toda la configuración de red activa en una sola ráfaga.
 * Útil para verificar que el robot está correctamente identificado en la red del laboratorio.
 */
void printConnectionInformation() 
{
  // 1. Identificación de la Red
  Serial.print("[INFO] SSID: ");
  Serial.println(WiFi.SSID()); // Nombre de la red WiFi

  // 2. Identificación del Router (BSSID)
  // Aquí imprimes byte por byte manualmente en lugar de usar una función auxiliar
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("[INFO] BSSID: ");
  // Nota: Imprime en orden inverso (de 5 a 0) para seguir el estándar de red
  Serial.print(bssid[5], HEX); Serial.print(":");
  Serial.print(bssid[4], HEX); Serial.print(":");
  Serial.print(bssid[3], HEX); Serial.print(":");
  Serial.print(bssid[2], HEX); Serial.print(":");
  Serial.print(bssid[1], HEX); Serial.print(":");
  Serial.println(bssid[0], HEX);

  // 3. Calidad de la Conexión
  long rssi = WiFi.RSSI();
  Serial.print("[INFO] Signal Strength (RSSI): ");
  Serial.println(rssi); // Si ves -90, el robot perderá paquetes de la Raspberry Pi

  // 4. Seguridad
  byte encryption = WiFi.encryptionType();
  Serial.print("[INFO] Encryption Type: ");
  Serial.println(encryption, HEX);

  // 5. Dirección de Red (La que usarías para SSH o Socket)
  IPAddress ip = WiFi.localIP();
  Serial.print("[INFO] IP Address: ");
  Serial.println(ip);

  // 6. Dirección Física del Robot
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("[INFO] MAC Address: ");
  Serial.print(mac[5], HEX); Serial.print(":");
  Serial.print(mac[4], HEX); Serial.print(":");
  Serial.print(mac[3], HEX); Serial.print(":");
  Serial.print(mac[2], HEX); Serial.print(":");
  Serial.print(mac[1], HEX); Serial.print(":");
  Serial.println(mac[0], HEX);
}

/**
 * Muestra detalles de la red WiFi a la que está conectado el robot.
 */
void printCurrentNet() {
  // Imprime el nombre de la red (SSID)
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Imprime la MAC del router (BSSID)
  // Útil si hay varios repetidores y quieres saber a cuál se conectó el robot
=======
// void serialEvent() {
//   static unsigned char* read_ptr = (unsigned char*)&packetBuffer;
//   static bool start_of_data = false;

//   while (Serial1.available()) {
//     char c = Serial1.read();
    
//    if (c == 'K') {  // Si se encuentra un carácter de nueva línea
//       // Agrega un terminador nulo para indicar el final de los datos
//       // Reinicia el puntero de datos para leer los datos desde el principio
//       read_ptr = (unsigned char*)&packetBuffer;
//       // Marca el final de los datos
//       server_operation = (struct appdata *)&packetBuffer;
//       serialCom = true;
//       start_of_data = false;  // Reinicia el marcador del inicio de los datos
//       DEBUG_PRINTLN(server_operation->len);
//     } 
//     else 
//     {
//       if (!start_of_data) 
//       {  // Si no se ha empezado a recibir datos, marca el inicio
//         start_of_data = true;
//         read_ptr = (unsigned char*)&packetBuffer;
//       }
      
//       *read_ptr++ = c;  // Almacena el byte en el buffer y avanza el puntero
//     }
//   }
  
// }

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);


  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
>>>>>>> 9fcf88d (Refactorización)
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

<<<<<<< HEAD
  // Intensidad de la señal (RSSI - Received Signal Strength Indication)
  // Valores cercanos a -30 son excelentes, -80 es una conexión muy débil
=======
  // print the received signal strength:
>>>>>>> 9fcf88d (Refactorización)
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

<<<<<<< HEAD
  // Tipo de seguridad (WPA2, WEP, etc.) en formato hexadecimal
=======
  // print the encryption type:
>>>>>>> 9fcf88d (Refactorización)
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

<<<<<<< HEAD

/**
 * Establece la conexión WiFi y sincroniza el cliente MQTT.
 * Esta función es bloqueante: el robot no empezará a moverse hasta estar conectado.
 */
void connect() {
  Serial.print("checking wifi...");
  
  // 1. BUCLE DE CONEXIÓN WIFI
  // Intenta conectar continuamente hasta que el estado sea WL_CONNECTED
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass); // Usa las credenciales definidas previamente
    Serial.print(".");
    delay(1000); // Espera 1 segundo entre intentos para no saturar el chip WiFi
  }
  Serial.println("\nconnected to WiFi!\n");
  
  // Muestra por el monitor serie toda la info que revisamos antes (IP, MAC, etc.)
  Serial.print("[INFO] Connection Successful");
  printConnectionInformation();
  Serial.println("-----------------------------------------------");

  // 2. CONEXIÓN AL BROKER MQTT
  // El broker es el servidor central que coordina los mensajes del enjambre
  Serial.print("Attempting MQTT connection ....");
  Serial.println(broker);
  
  if(!mqttClient.connect(broker, mqttPort))
  {
    // Si falla la conexión MQTT, el robot se detiene por seguridad (bucle infinito)
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1); 
  }
  
  Serial.println("You're connected to the MQTT broker!");

  // 3. MENSAJE DE SALUDO (Handshake IoT)
  // Construye una cadena en formato JSON para avisar al servidor que este robot está online
  String messagePayload = "{\"operation\": \"hello\", \"source_id\": \"arduinoClient\", \"payload\": {\"url\": \"example.com\"}}";
  
  // Envía el mensaje al tópico configurado por defecto
  mqttClient.beginMessage(""); // El argumento vacío suele ser el tópico de publicación
  mqttClient.print(messagePayload);
  // Nota: Falta mqttClient.endMessage() si la librería lo requiere para enviar el buffer.
}
#endif
=======
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}


/*
Connects to Wifi and MQTT Broker
*/
void connect() {
  Serial.print("checking wifi...");
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected to WiFi!\n");
  
  // Connection successful
  Serial.print("[INFO] Connection Successful");
  Serial.print("");  
  printConnectionInformation();
  Serial.println("-----------------------------------------------");
  Serial.println(""); 

  Serial.print("Attempting MQTT connection ....");
  Serial.println(broker);
  if(!mqttClient.connect(broker,mqttPort))
  {
    Serial.print("MQTT connection failed! Error code = ");

    Serial.println(mqttClient.connectError());


    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
 // Construct the operation message as a JSON string
  String messagePayload = "{\"operation\": \"hello\", \"source_id\": \"arduinoClient\", \"payload\": {\"url\": \"example.com\"}}";
  mqttClient.beginMessage("");
  mqttClient.print(messagePayload);
}



/*
Prints Wifi connection information
*/
void printConnectionInformation() 
{
  // Print Network SSID
  Serial.print("[INFO] SSID: ");
  Serial.println(WiFi.SSID());

  // Print Router's MAC address
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("[INFO] BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // Print received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("[INFO] Signal Strength (RSSI): ");
  Serial.println(rssi);

  // Print encryption type
  byte encryption = WiFi.encryptionType();
  Serial.print("[INFO] Encryption Type: ");
  Serial.println(encryption, HEX);

  // Print WiFi Shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("[INFO] IP Address: ");
  Serial.println(ip);

  // Print MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("[INFO] MAC Address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}
>>>>>>> 9fcf88d (Refactorización)
