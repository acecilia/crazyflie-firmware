#include "RandomizedTimerFunctions.h"
#include <stdlib.h>

static TickType_t ticksPerSecond = M2T(1000);

void vCallbackFunction(TimerHandle_t xTimer) {
  randomizedTimer_t* randomizedTimer = pvTimerGetTimerID(xTimer);
  start(randomizedTimer);

  randomizedTimer->callback();
}

void init(randomizedTimer_t* randomizedTimer, uint16_t frequency, void (*callback)(void)) {
  changeFrequency(randomizedTimer, frequency);
  randomizedTimer->callback = callback;

  // If FreeRTOS is updated to v9.0.0, it is possible to use xTimerCreateStatic instead of xTimerCreate
  randomizedTimer->timer = xTimerCreate("randomizedTimer", 1 /* value not used */, pdFALSE, randomizedTimer, vCallbackFunction);
}

TickType_t randomizePeriod(TickType_t averagePeriod) {
  return averagePeriod / 2 + rand() % averagePeriod;
}

void start(randomizedTimer_t* randomizedTimer) {
  TickType_t randomizedPeriod = randomizePeriod(randomizedTimer->averagePeriod);
  xTimerChangePeriod(randomizedTimer->timer, randomizedPeriod, 0);
}

void stop(randomizedTimer_t* randomizedTimer) {
  xTimerStop(randomizedTimer->timer, 0);
}

void changeFrequency(randomizedTimer_t* randomizedTimer, uint16_t frequency) {
  randomizedTimer->averagePeriod = ticksPerSecond / frequency;
}


