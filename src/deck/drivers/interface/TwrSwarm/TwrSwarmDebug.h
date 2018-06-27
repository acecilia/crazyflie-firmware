#ifndef TwrSwarmDebug_h
#define TwrSwarmDebug_h

#include <stdint.h>
#include "led.h"

typedef struct {
  float localRx;
  float localTx;
  float remoteRx;
  float remoteTx;

  uint32_t remoteReply;
  uint32_t localReply;
  uint32_t localRound;
  
  uint32_t clockCorrection;
  uint32_t clockCorrectionCandidate;
  uint32_t clockUpdated;
  uint32_t clockNotAccepted;
  uint32_t clockAcceptanceRate;

  uint32_t tof;

  uint32_t dctCount;

  uint16_t totalRangingPerSec;
  uint16_t succededRangingPerSec;

  uint32_t auxiliaryValue;
  
  uint32_t measurementFailure;
  uint32_t idFailure;
  uint32_t reflections;

  void (*blink)(led_t led);
  void (*init)(void);
} debug_t;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
extern debug_t debug;
#endif

#endif /* TwrSwarmDebug_h */
