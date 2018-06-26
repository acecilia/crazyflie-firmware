#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"
#include "randomizedTimerEngine.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

static lpsSwarmPacket_t packet;

/**
 A type that encapsulates all the required global values of the algorithm
 */
static struct {
  dwDevice_t* dev; // Needed when sending tx packets at random times

  dict* neighboursDct;
  dict* tofDct;

  locoId_t localId;
  uint8_t nextTxSeqNr; // Local sequence number of the transmitted packets

  // Add packet sequence number

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
  uint8_t numberOfNeighbours = dict_count(ctx.neighboursDct);
  ctx.averageTxDelay = calculateAverageTxDelay(numberOfNeighbours);
}

/**
 Initialize all the required variables of the algorithm
 */
static void init() {
  configure_dict_malloc();

  // Initialize the context
  ctx.neighboursDct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10); // Dictionary storing the neighbours data
  ctx.tofDct = hashtable2_dict_new(dict_uint16_cmp, dict_uint16_hash, 10); // Dictionary storing the tof data

  ctx.localId = generateId();
  ctx.nextTxSeqNr = 0;

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

  setTxData(txPacket, ctx.localId, &ctx.nextTxSeqNr, ctx.neighboursDct, ctx.tofDct);
  unsigned int packetSize = calculatePacketSize(txPacket);

  // Set tx time inside txPacket
  dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
  uint64_t localTx = tx.full;
  txPacket->header.tx = localTx;

  // Set data
  dwSetData(dev, (uint8_t*)txPacket, packetSize);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, tx);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

/**
 Handle the data coming in a received packet
 */
static void handleRxPacket(dwDevice_t *dev) {
  // Makes sure the id is unique among the neighbours around, and regenerate it only if the local ranging information is less than the information coming on the packet
  if (packet.header.sourceId == ctx.localId) {
    ctx.localId = generateIdNotIn(&packet, ctx.neighboursDct);
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
    debug.idFailure++;
#endif
  }

  processRxPacket(dev, ctx.localId, &packet, ctx.neighboursDct, ctx.tofDct);
}

/**
 Called for each DW radio event
 */
static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  unsigned int dataLength = 0;

  // If data was received, get it before changing the status of the DW1000 chip
  if (event == eventPacketReceived) {
    dataLength = dwGetDataLength(dev);
    dwGetData(dev, (uint8_t*)&packet, dataLength);
  }

  // Configure the DW1000 for Rx before processing the event: we want the chip on Rx mode as much time as possible, to avoid losing packets
  setupRx(dev);

  // Process the event
  if (event == eventPacketReceived && dataLength > 0) {
    handleRxPacket(dev);
  } else if (event == eventTimeout) {
    ctx.timeOfNextTx = now() + calculateRandomDelayToNextTx(ctx.averageTxDelay);
    transmit(dev);
  }

  int32_t timeoutForNextTx = (int32_t)(ctx.timeOfNextTx - now());
  if (timeoutForNextTx < 0) {
    // Force a timeout, that will result in a new call to onEvent
    timeoutForNextTx = 0;
  }
  return (uint32_t)timeoutForNextTx;
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .onEvent = onEvent
};
