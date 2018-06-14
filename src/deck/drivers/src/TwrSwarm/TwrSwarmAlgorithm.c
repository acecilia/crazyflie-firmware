#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"
#include "randomizedTimerEngine.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#include "debug.h"
#endif

/**
 A type that encapsulates all the required global values of the algorithm
 */
static struct {
  dict *dct;

  locoId_t localId;

  // Add packet sequence number

  // Values to calculate t_round
  uint64_t localTx; // To be set after transmission

  randomizedTimer_t randomizedTimer;
} ctx;

static void transmitCallback() {
  DEBUG_PRINT("transmit\n");
}

static void init() {
  configure_dict_malloc();

  // Initialize the context
  ctx.dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10); // Dictionary storing the rangings
  ctx.localId = generateId();
  ctx.localTx = 0;

  int16_t averageTxFrequency = calculateAverageTxFrequency(0);
  randomizedTimerEngine.init(&ctx.randomizedTimer, transmitCallback);
  randomizedTimerEngine.setFrequency(&ctx.randomizedTimer, averageTxFrequency);
  randomizedTimerEngine.start(&ctx.randomizedTimer);
}

static void initiateRanging(dwDevice_t *dev) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.totalRangingPerSec++;
#endif

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static uint32_t rxcallback(dwDevice_t *dev, lpsAlgoOptions_t* options, lpsSwarmPacket_t* rxPacket, unsigned int dataLength) {
  if (dataLength > 0) {
    // Makes sure the id is unique among the neighbours around, and regenerate it only if the local ranging information is less than the information coming on the packet
    if (rxPacket->sourceId == ctx.localId && dict_count(ctx.dct) <= rxPacket->payloadLength) {
      ctx.localId = generateIdNotInPacket(rxPacket);
    }

    uint8_t prevNumberOfNeighbours = dict_count(ctx.dct);

    processRxPacket(dev, ctx.localId, rxPacket, ctx.dct, ctx.localTx);

    uint8_t numberOfNeighbours = dict_count(ctx.dct);
    if (numberOfNeighbours != prevNumberOfNeighbours) {
      int16_t averageTxFrequency = calculateAverageTxFrequency(numberOfNeighbours);
      randomizedTimerEngine.setFrequency(&ctx.randomizedTimer, averageTxFrequency);
      randomizedTimerEngine.start(&ctx.randomizedTimer);
    }
  }

  if (true) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
    debug.totalRangingPerSec++;
#endif

    // Is its turn to send data

    // Create txPacket
    lpsSwarmPacket_t* txPacket;
    unsigned int txPacketLength = allocAndFillTxPacket(&txPacket, ctx.dct, ctx.localId);

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
  } else {
    // Has to wait for the next neighbour
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
  }

  return MAX_TIMEOUT;
}

static void txcallback(dwDevice_t *dev) {
  // DEBUG_PRINT("txcallback\n");
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .initiateRanging = initiateRanging,
  .rxcallback = rxcallback,
  .txcallback = txcallback
};
