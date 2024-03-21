#ifndef OPERATION_H
#define OPERATION_H


typedef enum Operation{
  OP_ERROR = 99,
  OP_HELLO = 1,
  OP_MOVE_ROBOT = 2,
  OP_STOP_ROBOT = 3,
  OP_VEL_ROBOT  = 4,
  OP_TURN_ROBOT = 5,
  OP_SILENCE    = 6,
  OP_SEND_TELEMETRY = 7,
  OP_POSITION   = 8,
  OP_CONF_PID   = 9,
  OP_CONF_FF    = 10,//FeedForward
  OP_DONE       = 11,
}operation_t;


operation_t operation;

#endif
