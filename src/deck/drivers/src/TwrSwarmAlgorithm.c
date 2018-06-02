#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

/**
 A type that encapsulates all the required global values of the algorithm
 */
static struct {
  dict *dct;

  // Values to calculate t_round
  uint64_t localTx; // To be set after transmission
} ctx;


static void init() {
  configure_dict_malloc();

  // Initialize the dictionary storing the rangings
  ctx.dct = hashtable2_dict_new(dict_uint64_cmp, dict_uint64_hash, 10);
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
    processRxPacket(dev, options->tagAddress, rxPacket, ctx.dct, ctx.localTx);
  }

  if (true) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
    debug.totalRangingPerSec++;
#endif

    // Is its turn to send data

    // Local values
    dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
    uint64_t localTx = tx.full;

    lpsSwarmPacket_t* txPacket;
    unsigned int txPacketLength = createTxPacket(&txPacket, ctx.dct, options->tagAddress, localTx);

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
