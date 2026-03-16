/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  operation.h
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Mapa de operaciones (OP_CODES). Define las funciones de respuesta para
 * comandos como giro, movimiento, configuración de PID y reportes de estado.
 * ----------------------------------------------------------------------------
 */
 
#ifndef OPERATION_H
#define OPERATION_H

/**
 * ENUMERACIÓN DE OPERACIONES
 * Define los códigos que la Raspberry Pi envía para comandar al Arduino.
 */
typedef enum Operation{
  OP_ERROR = 99,       // Notificación de error en el sistema
  OP_HELLO = 1,        // Saludo/Sincronización inicial
  OP_MOVE_ROBOT = 2,   // Mover el robot de forma combinada (v, w)
  OP_STOP_ROBOT = 3,   // Parada total inmediata
  OP_TELEMETRY  = 4,   // Envío de datos de sensores al servidor
  OP_TURN_ROBOT = 5,   // Rotación sobre el eje central
  OP_SILENCE    = 6,   // Detener envío de telemetría
  OP_POSITION   = 7,   // Actualizar o pedir posición actual
  OP_CONF_PID   = 8,   // Configuración de constantes Proporcional, Integral, Derivativa
  OP_CONF_FF    = 9,   // Configuración de parámetros FeedForward
  OP_DONE       = 10,  // Confirmación de tarea completada
  OP_MOVE_WHEELS = 11  // Control directo de velocidad por cada rueda individual
}operation_t;

// Variable global que almacena la operación que se está ejecutando actualmente
operation_t operation;

#endif
