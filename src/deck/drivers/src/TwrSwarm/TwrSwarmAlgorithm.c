#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"
#include "randomizedTimerEngine.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#include "debug.h"
#endif

static lpsSwarmPacket_t packet;

/**
 A type that encapsulates all the required global values of the algorithm
 */
static struct {
  dwDevice_t* dev; // Needed when sending tx packets at random times
  dict* dct;
  locoId_t localId;

  // Add packet sequence number

  // Values to calculate t_round
  uint64_t localTx; // To be set after transmission

  uint32_t timeOfNextTx;
  uint32_t averageTxDelay;

  xTimerHandle logTimer;
} ctx;

/* Helpers */
/**********************************/

static void setupRx(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

uint32_t now() {
  return xTaskGetTickCount();
}

/**********************************/

/**
 Callback for the timer. Used to update variables of the algorithm periodically
 */
static void timerCallback(xTimerHandle timer) {
  // Adjust average tx delay based on the number of known drones around
  uint8_t numberOfNeighbours = dict_count(ctx.dct);
  ctx.averageTxDelay = calculateAverageTxDelay(numberOfNeighbours);
}

/**
 Initialize all the required variables of the algorithm
 */
static void init() {
  configure_dict_malloc();

  // Initialize the context
  ctx.dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10); // Dictionary storing the rangings
  ctx.localId = generateId();
  ctx.localTx = 0;

  // Related with random transmission
  ctx.averageTxDelay = calculateAverageTxDelay(0);
  ctx.timeOfNextTx = calculateRandomDelayToNextTx(ctx.averageTxDelay);

  // Timer to execute actions periodically
  ctx.logTimer = xTimerCreate("timer", M2T(1000), pdTRUE, NULL, timerCallback);
  xTimerStart(ctx.logTimer, 0);
}

/**
 Fill and transmit a packet
 */
static void transmit(dwDevice_t *dev) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.totalRangingPerSec++;
#endif

  lpsSwarmPacket_t* txPacket = &packet;

  setTxData(txPacket, ctx.dct, ctx.localId);
  unsigned int packetSize = calculatePacketSize(txPacket);

  // Set tx time inside txPacket
  dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
  uint64_t localTx = tx.full;
  txPacket->header.tx = localTx;

  // Set data
  dwSetData(dev, (uint8_t*)txPacket, packetSize);
  vPortFree(txPacket);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, tx);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);

  // Set historic
  ctx.localTx = localTx;
}

/**
 Handle the data coming in a received packet
 */
static void handleRxPacket(dwDevice_t *dev) {
  // Makes sure the id is unique among the neighbours around, and regenerate it only if the local ranging information is less than the information coming on the packet
  if (packet.header.sourceId == ctx.localId) {
    ctx.localId = generateIdNotIn(&packet, ctx.dct);
  }

  processRxPacket(dev, ctx.localId, &packet, ctx.dct, ctx.localTx);
}

/**
 Called for each DW radio event
 */
static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  dwIdle(dev); // Put the chip in idle (to save battery during processing) when not receiving or transmitting

  if (event == eventTimeout) {
    ctx.timeOfNextTx = now() + calculateRandomDelayToNextTx(ctx.averageTxDelay);
    transmit(dev);
  } else if (event == eventPacketReceived) {
    unsigned int dataLength = dwGetDataLength(dev);
    dwGetData(dev, (uint8_t*)&packet, dataLength);

    // SetupRx after getting the packet data, and before processing it
    setupRx(dev);

    if (dataLength > 0) {
      handleRxPacket(dev);
    }
  } else {
    setupRx(dev);
  }

  int32_t timeoutForNextTx = ctx.timeOfNextTx - now();
  if (timeoutForNextTx <= 0) {
    // Force a timeout, that will result in a new call to onEvent
    timeoutForNextTx = 0;
  }
  return timeoutForNextTx;
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .onEvent = onEvent
};
