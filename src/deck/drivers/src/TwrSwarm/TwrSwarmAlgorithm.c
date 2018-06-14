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
} ctx;

/* Helpers */
/**********************************/

static void setupRx(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

/**********************************/

static void transmit(dwDevice_t *dev) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.totalRangingPerSec++;
#endif

  // Create txPacket
  lpsSwarmPacket_t* txPacket;
  unsigned int txPacketLength = allocAndFillTxPacket(&packet, ctx.dct, ctx.localId);

  // Set tx time inside txPacket
  dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
  uint64_t localTx = tx.full;
  txPacket->tx = localTx;

  // Set data
  dwSetData(dev, (uint8_t*)txPacket, txPacketLength);
  vPortFree(txPacket);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, tx);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);

  // Set historic
  ctx.localTx = localTx;
}

static void init() {
  configure_dict_malloc();

  // Initialize the context
  ctx.dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10); // Dictionary storing the rangings
  ctx.localId = generateId();
  ctx.localTx = 0;

  /*
  int16_t averageTxFrequency = calculateAverageTxFrequency(0);
  randomizedTimerEngine.init(&ctx.randomizedTimer, transmitCallback);
  randomizedTimerEngine.setFrequency(&ctx.randomizedTimer, averageTxFrequency);
  randomizedTimerEngine.start(&ctx.randomizedTimer);
   */
}

static void handleRxPacket(dwDevice_t *dev) {
  // Makes sure the id is unique among the neighbours around, and regenerate it only if the local ranging information is less than the information coming on the packet
  if (packet.sourceId == ctx.localId && dict_count(ctx.dct) <= packet.payloadLength) {
    ctx.localId = generateIdNotInPacket(&packet);
  }

  uint8_t prevNumberOfNeighbours = dict_count(ctx.dct);

  processRxPacket(dev, ctx.localId, &packet, ctx.dct, ctx.localTx);

  uint8_t numberOfNeighbours = dict_count(ctx.dct);
  if (numberOfNeighbours != prevNumberOfNeighbours) {
    int16_t averageTxFrequency = calculateAverageTxFrequency(numberOfNeighbours);
    randomizedTimerEngine.setFrequency(&ctx.randomizedTimer, averageTxFrequency);
    randomizedTimerEngine.start(&ctx.randomizedTimer);
  }
}

/**
 Called for each DW radio event
 */
static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  bool shouldHandleRxPacket = false;

  if (event == eventPacketReceived) {
    unsigned int dataLength = dwGetDataLength(dev);
    if (dataLength > 0) {
      dwGetData(dev, (uint8_t*)&packet, dataLength);
      shouldHandleRxPacket = true;
    }
  }

  setupRx(dev);

  if (shouldHandleRxPacket) {
    handleRxPacket(dev);
  }

  uint32_t now = xTaskGetTickCount();
  int32_t timeoutForNextTx = ctx.timeOfNextTx - now;
  if (timeoutForNextTx <= 0) {
    ctx.timeOfNextTx = now + calculateRandomDelayToNextTx(ctx.averageTxDelay);
    timeoutForNextTx = ctx.timeOfNextTx - now;

    transmit(dev);
  }
  return timeoutForNextTx;
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .onEvent = onEvent
};
