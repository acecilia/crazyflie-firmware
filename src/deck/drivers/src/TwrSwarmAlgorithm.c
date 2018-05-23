#include "TwrSwarmAlgorithm.h"

#include "debug.h" // To be removed?

ctx_s ctx;

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

// Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0
static uint32_t adjustTxRxTime(dwTime_t *time) {
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));

  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);

  return added;
}

static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev) {
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // And some extra
  transmitTime.full += TDMA_EXTRA_LENGTH;

  // TODO krri Adding randomization on this level adds a long delay, is it worth it?
  // The randomization on OS level is quantized to 1 ms (tick time of the system)
  // Add a high res random to smooth it out
  // uint32_t r = rand();
  // uint32_t delay = r % TDMA_HIGH_RES_RAND;
  // transmitTime.full += delay;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);

  return transmitTime;
}

/*static double calculateClockCorrection(uint32_t prevRemoteTx, uint32_t remoteTx, uint32_t prevLocalRx, uint32_t localRx) {
  if (prevRemoteTx == 0 || remoteTx == 0 || prevLocalRx == 0 || localRx == 0) {
    return 1;
  }

  double result = 1;

  // Assigning to uint32_t truncates the diffs and takes care of wrapping clocks
  uint32_t tickCountRemote = remoteTx - prevRemoteTx;
  uint32_t tickCountLocal = localRx - prevLocalRx;

  if (tickCountRemote != 0) {
    result = (double)tickCountLocal / (double)tickCountRemote;
  }

  return result;
}*/

static neighbourData_t* getDataForNeighbour(dict* dct, locoAddress_t address) {
  void** search_result = dict_search(dct, &address);
  if (search_result) {
    return (neighbourData_t *)*search_result;
  } else {
    locoAddress_t* key = pvPortMalloc(sizeof(locoAddress_t));
    *key = address;

    neighbourData_t* data = pvPortMalloc(sizeof(neighbourData_t));
    data->localRx = 0;
    data->remoteTx = 0;
    data->tof = 0;

    dict_insert_result insert_result = dict_insert(dct, key);
    *insert_result.datum_ptr = data;
    return data;
  }
}

static unsigned int createTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoAddress_t sourceAddress, uint32_t localTx) {
  // Packet creation
  unsigned int rxLength = dict_count(dct);
  unsigned int txPacketLength = sizeof(lpsSwarmPacket_t) + rxLength * sizeof(addressTimePair_t);

  lpsSwarmPacket_t* txPacket = pvPortMalloc(txPacketLength);
  *txPacketPointer = txPacket;
  txPacket->sourceAddress = sourceAddress;
  txPacket->tx = localTx;
  txPacket->rxLength = rxLength;

  if (rxLength > 0) {
    // Get data from the dict and into the txPacket array
    dict_itor *itor = dict_itor_new(dct);
    dict_itor_first(itor);
    for (int i = 0; i < rxLength; i++) {
      locoAddress_t key = *(locoAddress_t*)dict_itor_key(itor);
      neighbourData_t* data = (neighbourData_t*)*dict_itor_datum(itor);
      addressTimePair_t pair = {
        .address = key,
        .time = data->localRx
      };

      ////////////
      DEBUG_PRINT("****Send => address: %lld; time: %ld****\n", pair.address, pair.time);
      ////////////

      txPacket->rx[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }

  return txPacketLength;
}

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
