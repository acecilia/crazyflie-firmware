#include "randomizedTimerEngine.h"
#include "randomizedTimerFunctions.h"

randomizedTimerEngine_t randomizedTimerEngine = {
  .init = init,
  .start = start,
  .stop = stop,
  .setFrequency = setFrequency
};
