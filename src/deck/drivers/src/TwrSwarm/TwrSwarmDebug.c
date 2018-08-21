#include "TwrSwarmDebug.h"
#include "TwrSwarmDump.h"
#include "log.h"
#include "param.h"

#include "FreeRTOS.h"
#include "timers.h"

static void blink(led_t led);
static void init();

debug_t debug = {
  .position = {
    .timestamp = 0,
    .x = 0.1,
    .y = 0.2,
    .z = 0.3
  },

  .position0 = {
    .timestamp = 0,
    .x = 0.4,
    .y = 0.5,
    .z = 0.6
  },

  .position1 = {
    .timestamp = 0,
    .x = 0.7,
    .y = 0.8,
    .z = 0.9
  },

  .localRx = 100,
  .localTx = 200,
  .remoteRx = 300,
  .remoteTx = 400,

  .remoteReply = 1000,
  .localReply = 2000,
  .localRound = 3000,

  .clockCorrection = 3800,
  .clockCorrectionCandidate = 3900,
  .clockUpdated = 0,
  .clockNotAccepted = 0,
  .clockAcceptanceRate = 0,

  .tof = 4000,
  .distance = 4100,

  .neighbourCount = 5000,

  .sentPacketsPerSec = 6000,
  .receivedPacketsPerSec = 6200,
  .succededTofCalculationPerSec = 6300,

  .auxiliaryValue = 8000, // To log something fast without the need of created the necessary logging code for it

  .measurementFailure = 0,
  .idFailure = 0,
  .reflections = 0,

  .drone = 0,

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
uint16_t lastKnownSentPacketsPerSec = 0;
uint16_t lastKnownExpectedReceivedPacketsPerSec = 0;
uint16_t lastKnownReceivedPacketsPerSec  = 0;
uint16_t lastKnownSuccededTofCalculationPerSec = 0;

// Timer debug tools
static xTimerHandle logTimer;
static void logTimerCallback(xTimerHandle timer) {
  lastKnownSentPacketsPerSec = debug.sentPacketsPerSec;
  lastKnownExpectedReceivedPacketsPerSec = debug.sentPacketsPerSec * debug.neighbourCount;
  lastKnownReceivedPacketsPerSec = debug.receivedPacketsPerSec;
  lastKnownSuccededTofCalculationPerSec = debug.succededTofCalculationPerSec;

  debug.sentPacketsPerSec = 0;
  debug.receivedPacketsPerSec = 0;
  debug.succededTofCalculationPerSec = 0;

  debug.clockAcceptanceRate = (debug.clockUpdated - debug.clockNotAccepted) * 100 / debug.clockUpdated;
  debug.clockUpdated = 0;
  debug.clockNotAccepted = 0;
}

static void init() {
  logTimer = xTimerCreate("loggingTimer", M2T(1000), pdTRUE, NULL, logTimerCallback);
  xTimerStart(logTimer, 0);

  twrSwarmDumpInit();
}

// Stock log groups
LOG_GROUP_START(kalman)
LOG_ADD(LOG_FLOAT, stateX, &debug.position.x)
LOG_ADD(LOG_FLOAT, stateY, &debug.position.y)
LOG_ADD(LOG_FLOAT, stateZ, &debug.position.z)
LOG_GROUP_STOP(kalman)

LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_FLOAT, xPosition, &debug.position.x)
LOG_ADD(LOG_FLOAT, yPosition, &debug.position.y)
LOG_ADD(LOG_FLOAT, zPosition, &debug.position.z)

LOG_ADD(LOG_FLOAT, xPosition0, &debug.position0.x)
LOG_ADD(LOG_FLOAT, yPosition0, &debug.position0.y)
LOG_ADD(LOG_FLOAT, zPosition0, &debug.position0.z)

LOG_ADD(LOG_FLOAT, xPosition1, &debug.position1.x)
LOG_ADD(LOG_FLOAT, yPosition1, &debug.position1.y)
LOG_ADD(LOG_FLOAT, zPosition1, &debug.position1.z)

LOG_ADD(LOG_FLOAT, localRx, &debug.localRx)
LOG_ADD(LOG_FLOAT, localTx, &debug.localTx)
LOG_ADD(LOG_FLOAT, remoteRx, &debug.localRx)
LOG_ADD(LOG_FLOAT, remoteTx, &debug.localTx)

LOG_ADD(LOG_UINT32, remoteReply, &debug.remoteReply)
LOG_ADD(LOG_UINT32, localReply, &debug.localReply)
LOG_ADD(LOG_UINT32, localRound, &debug.localRound)

LOG_ADD(LOG_UINT32, ckCorr, &debug.clockCorrection)
LOG_ADD(LOG_UINT32, ckCorrCandidate, &debug.clockCorrectionCandidate)
LOG_ADD(LOG_UINT32, clockAcceptance, &debug.clockAcceptanceRate)

LOG_ADD(LOG_UINT32, tof, &debug.tof)
LOG_ADD(LOG_FLOAT, distance, &debug.distance)

LOG_ADD(LOG_UINT32, neighbourCount, &debug.neighbourCount)

LOG_ADD(LOG_UINT16, txPerSec, &lastKnownSentPacketsPerSec)
LOG_ADD(LOG_UINT16, expectedRxPerSec, &lastKnownExpectedReceivedPacketsPerSec)
LOG_ADD(LOG_UINT16, rxPerSec, &lastKnownReceivedPacketsPerSec)
LOG_ADD(LOG_UINT16, okTofPerSec, &lastKnownSuccededTofCalculationPerSec)

LOG_ADD(LOG_UINT32, auxiliaryValue, &debug.auxiliaryValue)

LOG_ADD(LOG_UINT32, measurementFail, &debug.measurementFailure)
LOG_ADD(LOG_UINT32, idFail, &debug.idFailure)
LOG_ADD(LOG_UINT32, reflections, &debug.reflections)
LOG_GROUP_STOP(twrSwarm)

PARAM_GROUP_START(twrSwarm)
PARAM_ADD(PARAM_UINT8, drone, &debug.drone)
PARAM_GROUP_STOP(twrSwarm)
