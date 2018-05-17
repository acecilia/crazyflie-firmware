#include "TwrSwarmAlgorithm.h"

#include "libdict.h"
#include "debug.h" // To be removed?

///// Debug, to be removed
#include "timers.h"
#include "log.h"
///// Debug, to be removed

static dict *dct = NULL;
// static dwTime_t timeOfLastSentPackage;
// static dwTime_t timeOfLastReceivedPackage;

///// Debug, to be removed
static xTimerHandle debugTimer;
static void timerCallback(xTimerHandle timer) {

}
static uint32_t rxCallbackExecution = 0;
static uint32_t sendingDelay = 0;

dwTime_t rxStartTime = { .full = 0 };
dwTime_t rxEndTime = { .full = 0 };
dwTime_t txTime = { .full = 0 };
///// Debug, to be removed

// static lpsSwarmPacket_t lpsSwarmPacket;

/*
dwTime_t departure;
dwGetTransmitTimestamp(dev, &departure);
departure.full += (options->antennaDelay / 2);

switch (txPacket.payload[0]) {
  case LPS_TWR_POLL:
    poll_tx = departure;
    break;
  case LPS_TWR_FINAL:
    final_tx = departure;
    break;
}
 */

static void init() {
  configure_dict_malloc();

  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10);

  // Initialize the debug timer
  debugTimer = xTimerCreate("debugTimer", M2T(500), pdTRUE, NULL, timerCallback);
  xTimerStart(debugTimer, 0);
}

static void initiateRanging(dwDevice_t *dev) {
  dwNewTransmit(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static uint32_t rxcallback(dwDevice_t *dev, lpsSwarmPacket_t *packet, lpsAlgoOptions_t* options) {
  ///// Debug, to be removed
  dwGetSystemTimestamp(dev, &rxStartTime);
  ///// Debug, to be removed


  // dwGetReceiveTimestamp(dev, &timeOfLastReceivedPackage);

  lpsSwarmPacket_t txPacket;
  txPacket.sourceAddress = options->tagAddress;
  txPacket.processingTime = 69;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, sizeof(lpsSwarmPacket_t));

  ///// Debug, to be removed
  dwGetSystemTimestamp(dev, &rxEndTime);
  ///// Debug, to be removed

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  return MAX_TIMEOUT;
}

static void txcallback(dwDevice_t *dev) {
  ///// Debug, to be removed
  dwGetTransmitTimestamp(dev, &txTime);

  rxCallbackExecution = rxEndTime.low32 - rxStartTime.low32;
  sendingDelay = txTime.low32 - rxEndTime.low32;
  ///// Debug, to be removed

  //dwGetTransmitTimestamp(dev, &timeOfLastSentPackage);
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .initiateRanging = initiateRanging,
  .rxcallback = rxcallback,
  .txcallback = txcallback
};

LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_UINT32, rxCallback, &rxCallbackExecution)
LOG_ADD(LOG_UINT32, sendingDelay, &sendingDelay)
LOG_GROUP_STOP(twrSwarm)
