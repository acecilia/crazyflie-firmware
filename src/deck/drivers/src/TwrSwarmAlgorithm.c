#include "TwrSwarmAlgorithm.h"

#include "libdict.h"
#include "debug.h" // To be removed?

static dict *dct = NULL;
static dwTime_t timeOfLastSentPackage;
static dwTime_t timeOfLastReceivedPackage;


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
}

static void initiateRanging(dwDevice_t *dev) {
  dwNewTransmit(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  DEBUG_PRINT("initiateRanging\n");
}

static uint32_t rxcallback(dwDevice_t *dev, lpsSwarmPacket_t *packet, lpsAlgoOptions_t* options) {
  DEBUG_PRINT("rxcallback: %d\n", (int)packet->processingTime);


  lpsSwarmPacket_t txPacket;

  if (timeOfLastSentPackage && timeOfLastReceivedPackage) {
    txPacket.timeSinceLastSentPackage = 
  }
  txPacket.sourceAddress = options->tagAddress;
  txPacket.processingTime = 69;

  dwGetReceiveTimestamp(dev, &timeOfLastReceivedPackage);


  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, sizeof(lpsSwarmPacket_t));

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  return MAX_TIMEOUT;
}

static void txcallback(dwDevice_t *dev) {
  DEBUG_PRINT("txcallback\n");

  dwGetTransmitTimestamp(dev, &timeOfLastSentPackage);
}


twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .initiateRanging = initiateRanging,
  .rxcallback = rxcallback,
  .txcallback = txcallback
};
