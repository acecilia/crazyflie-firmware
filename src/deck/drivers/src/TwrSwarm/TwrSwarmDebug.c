#include "TwrSwarmDebug.h"
#include "log.h"

#include "FreeRTOS.h"
#include "timers.h"

static void blink(led_t led);
static void init();

debug_t debug = {
  .localRx = 100,
  .localTx = 200,
  .remoteRx = 300,
  .remoteTx = 400,

  .remoteReply = 1000,
  .localReply = 2000,
  .localRound = 3000,
  .clockCorrection = 3800,
  .clockCorrectionCandidate = 3900,

  .tof = 4000,

  .dctCount = 5000,

  .totalRangingPerSec = 6000,
  .succededRangingPerSec = 7000,

  .auxiliaryValue = 8000, // To log something fast without the need of created the necessary logging code for it

  .measurementFailure = 0,
  .idFailure = 0,

  .blink = blink,
  .init = init
};

// Blink debug tools
static unsigned int BLUE_L_counter = 0;
static unsigned int GREEN_L_counter = 0;
static unsigned int RED_L_counter = 0;
static unsigned int GREEN_R_counter = 0;
static unsigned int RED_R_counter = 0;

static void blink(led_t led) {
  unsigned int* blinkCounter = NULL;

  switch (led) {
    case LED_BLUE_L:
      blinkCounter = &BLUE_L_counter;
      break;
    case LED_GREEN_L:
      blinkCounter = &GREEN_L_counter;
      break;
    case LED_RED_L:
      blinkCounter = &RED_L_counter;
      break;
    case LED_GREEN_R:
      blinkCounter = &GREEN_R_counter;
      break;
    case LED_RED_R:
      blinkCounter = &RED_R_counter;
      break;
  }

  (*blinkCounter)++;

  if (*blinkCounter == 1000) {
    *blinkCounter = 0;
    ledToggle(led);
  }
}

// Ranging measurement tools
uint16_t lastKnownTotalRangingPerSec = 0;
uint16_t lastKnownSuccededRangingPerSec = 0;

// Timer debug tools
static xTimerHandle logTimer;
static void logTimerCallback(xTimerHandle timer) {
  lastKnownTotalRangingPerSec = debug.totalRangingPerSec;
  lastKnownSuccededRangingPerSec = debug.succededRangingPerSec;
  debug.totalRangingPerSec = 0;
  debug.succededRangingPerSec = 0;
}

static void init() {
  logTimer = xTimerCreate("loggingTimer", M2T(1000), pdTRUE, NULL, logTimerCallback);
  xTimerStart(logTimer, 0);
}

LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_FLOAT, localRx, &debug.localRx)
LOG_ADD(LOG_FLOAT, localTx, &debug.localTx)
LOG_ADD(LOG_FLOAT, remoteRx, &debug.localRx)
LOG_ADD(LOG_FLOAT, remoteTx, &debug.localTx)

LOG_ADD(LOG_UINT32, remoteReply, &debug.remoteReply)
LOG_ADD(LOG_UINT32, localReply, &debug.localReply)
LOG_ADD(LOG_UINT32, localRound, &debug.localRound)
LOG_ADD(LOG_UINT32, ckCorr, &debug.clockCorrection)
LOG_ADD(LOG_UINT32, ckCorrCandidate, &debug.clockCorrectionCandidate)

LOG_ADD(LOG_UINT32, tof, &debug.tof)

LOG_ADD(LOG_UINT32, dctCount, &debug.dctCount)

LOG_ADD(LOG_UINT16, rangingPerSec, &lastKnownTotalRangingPerSec)
LOG_ADD(LOG_UINT16, okRangingPerSec, &lastKnownSuccededRangingPerSec)

LOG_ADD(LOG_UINT32, auxiliaryValue, &debug.auxiliaryValue)

LOG_ADD(LOG_UINT32, measurementFail, &debug.measurementFailure)
LOG_ADD(LOG_UINT32, idFail, &debug.idFailure)
LOG_GROUP_STOP(twrSwarm)
