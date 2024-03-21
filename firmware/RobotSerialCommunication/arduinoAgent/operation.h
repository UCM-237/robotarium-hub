#ifndef OPERATION_H
#define OPERATION_H


typedef enum Operation{
  OP_ERROR = 99,
  OP_HELLO = 1,
  OP_MOVE_ROBOT = 2,
  OP_STOP_ROBOT = 3,
  OP_VEL_ROBOT  = 4,
  OP_POSITION   = 8,
  OP_CONF_PID   = 10,
  OP_CONF_FF    = 11,//FeedForward
   
}operation_t;


operation_t operation;

#endif
