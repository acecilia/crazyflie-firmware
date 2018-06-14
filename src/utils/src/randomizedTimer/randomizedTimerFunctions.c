#include "randomizedTimerFunctions.h"
#include <stdlib.h>

static TickType_t ticksPerSecond = M2T(1000);

void init(randomizedTimer_t* randomizedTimer, uint16_t frequency, void (*callback)(void)) {
  setFrequency(randomizedTimer, frequency);
  randomizedTimer->callback = callback;

  // If FreeRTOS is updated to v9.0.0, it is possible to use xTimerCreateStatic instead of xTimerCreate
  randomizedTimer->timer = xTimerCreate("randomizedTimer", 1 /* value not used */, pdFALSE, randomizedTimer, vCallbackFunction);
}

void start(randomizedTimer_t* randomizedTimer) {
  TickType_t randomizedPeriod = randomizePeriod(randomizedTimer->averagePeriod);
  xTimerChangePeriod(randomizedTimer->timer, randomizedPeriod, 0);
}

void stop(randomizedTimer_t* randomizedTimer) {
  xTimerStop(randomizedTimer->timer, 0);
}

void setFrequency(randomizedTimer_t* randomizedTimer, uint16_t frequency) {
  randomizedTimer->averagePeriod = ticksPerSecond / frequency;
}

void vCallbackFunction(TimerHandle_t xTimer) {
  randomizedTimer->callback();

  randomizedTimer_t* randomizedTimer = pvTimerGetTimerID(xTimer);
  start(randomizedTimer);
}

TickType_t randomizePeriod(TickType_t averagePeriod) {
  return averagePeriod / 2 + rand() % averagePeriod;
}

