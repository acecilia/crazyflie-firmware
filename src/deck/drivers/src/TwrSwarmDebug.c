#include "TwrSwarmDebug.h"
#include "log.h"

#include "FreeRTOS.h"
#include "timers.h"

static void blink(led_t led);
static void init();

debug_t debug = {
  .remoteReply = 1000,
  .remoteRx = 1100,
  .remoteTx = 1200,

  .localReply = 2000,

  .localRound = 3000,
  .localRx = 3100,
  .localTx = 3200,

  .tof = 4000,
  .dctCount = 5000,
  .totalRangingPerSec = 6000,
  .succededRangingPerSec = 7000,
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
LOG_ADD(LOG_UINT32, remoteReply, &debug.remoteReply)
LOG_ADD(LOG_UINT32, remoteRx, &debug.remoteRx)
LOG_ADD(LOG_UINT32, remoteTx, &debug.remoteTx)

LOG_ADD(LOG_UINT32, localReply, &debug.localReply)

LOG_ADD(LOG_UINT32, localRound, &debug.localRound)
LOG_ADD(LOG_UINT32, localRx, &debug.localRx)
LOG_ADD(LOG_UINT32, localTx, &debug.localTx)

LOG_ADD(LOG_UINT32, tof, &debug.tof)
LOG_ADD(LOG_UINT32, dctCount, &debug.dctCount)
LOG_ADD(LOG_UINT16, rangingPerSec, &lastKnownTotalRangingPerSec)
LOG_ADD(LOG_UINT16, okRangingPerSec, &lastKnownSuccededRangingPerSec)
LOG_GROUP_STOP(twrSwarm)
