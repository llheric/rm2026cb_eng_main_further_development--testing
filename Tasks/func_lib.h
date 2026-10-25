#ifndef _FUNC_LIB_H_
#define _FUNC_LIB_H_

#include "components.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "pid.h"
#include "fsfc.h"
#include "communicate.h"
#include "protocol.h"
#include "comm.h"
#include "fifo.h"
#include "led.h"

//homemade math functions
#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define cushionk(a, b, k) ((a) + (k) * ((b) - (a)))

#endif
