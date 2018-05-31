#ifndef TwrSwarmDebug_h
#define TwrSwarmDebug_h

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include <stdint.h>
#include "led.h"

typedef struct {
  uint32_t remoteReply;
  uint32_t remoteRx;
  uint32_t remoteTx;

  uint32_t localReply;

  uint32_t localRound;
  uint32_t localRx;
  uint32_t localTx;

  uint32_t tof;
  uint32_t dctCount;
  uint16_t totalRangingPerSec;
  uint16_t succededRangingPerSec;
  
  void (*blink)(led_t led);
  void (*init)();
} debug_t;

extern debug_t debug;
#endif

#endif /* TwrSwarmDebug_h */
