#ifndef randomizedTimerFunctions_h
#define randomizedTimerFunctions_h

#include "randomizedTimer.h"

/* Public */
void init(randomizedTimer_t* randomizedTimer, uint16_t frequency, void (*callback)(void));
void start(randomizedTimer_t* randomizedTimer);
void stop(randomizedTimer_t* randomizedTimer);
void changeFrequency(randomizedTimer_t* randomizedTimer, uint16_t frequency);

/* Private */
void vCallbackFunction(TimerHandle_t xTimer);
TickType_t randomizePeriod(TickType_t averagePeriod);

#endif /* randomizedTimerFunctions_h */
