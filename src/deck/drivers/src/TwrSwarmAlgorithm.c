#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"

ctx_s ctx;

static void init() {
  configure_dict_malloc();

  // Initialize the dictionary storing the rangings
  ctx.dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10);
}

static void initiateRanging(dwDevice_t *dev) {
  dwNewTransmit(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static uint32_t rxcallback(dwDevice_t *dev, lpsAlgoOptions_t* options, lpsSwarmPacket_t* rxPacket, unsigned int dataLength) {
  if (dataLength > 0) {
    // Process incoming package

    neighbourData_t* neighbourData = getDataForNeighbour(ctx.dct, rxPacket->sourceAddress);
    DEBUG_PRINT("neigh %ld", neighbourData->localRx);

    /*
    // Calculate Tof if the necessary data is available
    for(int i = 0; i < rxPacket->rxLength; i++) {
      if (rxPacket->rx[i].address == options->tagAddress) { // To be executed only once
        dwTime_t rx = { .full = 0 };
        dwGetReceiveTimestamp(dev, &rx);

        // Remote values
        uint32_t remoteRx = rxPacket->rx[i].time;
        uint32_t remoteTx = rxPacket->tx;
        uint32_t prevRemoteTx = neighbourData->remoteTx;

        // Local values
        uint32_t localRx = rx.low32;
        uint32_t localTx = ctx.localTx;
        uint32_t prevLocalRx = neighbourData->localRx;

        // Calculations
        uint32_t remoteReply = remoteTx - remoteRx;
        double clockCorrection = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);
        uint32_t localReply = remoteReply * clockCorrection;

        uint32_t localRound = localRx - localTx;
        neighbourData->tof = (localRound - localReply) / 2;

        // Set historic
        neighbourData->remoteTx = remoteTx;
        neighbourData->localRx = localRx;
        break;
      }
    }*/
  }

  if (true) {
    // Is its turn to send data

    // Local values
    dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
    uint32_t localTx = tx.low32;

    lpsSwarmPacket_t* txPacket = NULL;
    unsigned int txPacketLength = createTxPacket(&txPacket, ctx.dct, localTx, options->tagAddress);

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
  .txcallback = txcallback,

  // Exposed for testing
  .getDataForNeighbour = getDataForNeighbour,
  .ctx = &ctx
};
