#ifndef randomizedTimer_h
#define randomizedTimer_h

#include <stdint.h>
#include "FreeRTOS.h"
#include "timers.h"

typedef struct {
  xTimerHandle timer;
  TickType_t averagePeriod;
  void (*callback)(void);
} randomizedTimer_t;


#endif /* randomizedTimer_h */
