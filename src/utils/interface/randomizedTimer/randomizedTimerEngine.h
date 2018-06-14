/**
 Exposes the public API used for randomizedTimer in an Object Oriented way (and hides the functions that are only part of the internal implementation).
 */

#ifndef randomizedTimerEngine_h
#define randomizedTimerEngine_h

#include "randomizedTimer.h"

typedef struct {
  void (*init)(randomizedTimer_t* randomizedTimer, void (*callback)(void));
  void (*start)(randomizedTimer_t* randomizedTimer);
  void (*stop)(randomizedTimer_t* randomizedTimer);
  void (*setFrequency)(randomizedTimer_t* randomizedTimer, uint16_t frequency);
} randomizedTimerEngine_t;

extern randomizedTimerEngine_t randomizedTimerEngine;

#endif /* randomizedTimerEngine_h */
