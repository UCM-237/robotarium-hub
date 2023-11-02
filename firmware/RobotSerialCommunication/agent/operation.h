#ifndef OPERATION_H
#define OPERATION_H


enum Operation{
  OP_ERROR = 99,
  OP_SALUDO = 1,
  OP_MOVE_WHEEL = 2,
  OP_STOP_WHEEL = 3,
  OP_VEL_ROBOT  = 5,
  OP_POSITION   = 8,
  OP_CONF_PID   = 10,
   
};


Operation operation;

#endif
