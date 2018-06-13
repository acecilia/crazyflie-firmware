/**
 Exposes the randomizedTimer_t type, that represents the storage required for working with a randomizedTimer.
 */

#ifndef randomizedTimer_h
#define randomizedTimer_h

#include "FreeRTOS.h"
#include "timers.h"

typedef struct {
  xTimerHandle timer;
  TickType_t averagePeriod;
  void (*callback)(void);
} randomizedTimer_t;


#endif /* randomizedTimer_h */
