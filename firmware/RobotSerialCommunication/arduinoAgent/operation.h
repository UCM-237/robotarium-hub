#ifndef OPERATION_H
#define OPERATION_H


typedef enum Operation{
  OP_ERROR = 99,
  OP_HELLO = 1,
  OP_MOVE_ROBOT = 2,
  OP_STOP_ROBOT = 3,
  OP_TELEMETRY  = 4,
  OP_TURN_ROBOT = 5,
  OP_SILENCE    = 6,
  OP_POSITION   = 7,
  OP_CONF_PID   = 8,
  OP_CONF_FF    =9,//FeedForward
  OP_DONE       = 10,
}operation_t;


operation_t operation;

#endif
