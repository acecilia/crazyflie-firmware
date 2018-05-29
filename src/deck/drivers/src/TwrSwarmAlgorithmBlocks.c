#include "TwrSwarmAlgorithmBlocks.h"

// DEBUG (to be removed)
#include "debug.h"
#include "log.h"

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

// DEBUG (to be removed)
uint32_t remoteReply_db = 1000;
uint32_t localReply_db = 2000;
uint32_t localRound_db = 3000;
uint32_t tof_db = 4000;
uint32_t dctCount_db = 5000;

// Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0, and round the result up
uint32_t adjustTxRxTime(dwTime_t *time) {
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));

  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);

  return added;
}

dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev) {
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

double calculateClockCorrection(uint64_t prevRemoteTx, uint64_t remoteTx, uint64_t prevLocalRx, uint64_t localRx) {
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
}

neighbourData_t* getDataForNeighbour(dict* dct, locoAddress_t address) {
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

unsigned int createTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoAddress_t sourceAddress, uint64_t localTx) {
  // Packet creation
  unsigned int rxLength = dict_count(dct);
  unsigned int txPacketLength = sizeof(lpsSwarmPacket_t) + rxLength * sizeof(addressTimePair_t);

  *txPacketPointer = pvPortMalloc(txPacketLength);

  lpsSwarmPacket_t* txPacket = *txPacketPointer; // Declared for convenience and cleaner code
  txPacket->sourceAddress = sourceAddress;
  txPacket->tx = localTx;
  txPacket->rxLength = rxLength;

  if (rxLength > 0) {
    // Get data from the dict and into the txPacket array
    dict_itor *itor = dict_itor_new(dct);
    dict_itor_first(itor);
    for (unsigned int i = 0; i < rxLength; i++) {
      locoAddress_t key = *(locoAddress_t*)dict_itor_key(itor);
      neighbourData_t* data = (neighbourData_t*)*dict_itor_datum(itor);
      addressTimePair_t pair = {
        .address = key,
        .time = data->localRx
      };

      txPacket->rx[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }

  return txPacketLength;
}

void processRxPacket(dwDevice_t *dev, locoAddress_t ownAddress, lpsSwarmPacket_t* rxPacket, ctx_s* ctx) {
  dwTime_t rxTimestamp = { .full = 0 };
  dwGetReceiveTimestamp(dev, &rxTimestamp);
  neighbourData_t* neighbourData = getDataForNeighbour(ctx->dct, rxPacket->sourceAddress);

  // Remote values
  uint64_t remoteTx = rxPacket->tx;

  // Local values
  uint64_t localRx = rxTimestamp.full;

  for(int i = 0; i < rxPacket->rxLength; i++) {
    if (rxPacket->rx[i].address == ownAddress) { // To be executed only once

      // Remote values
      uint64_t remoteRx = rxPacket->rx[i].time;
      uint64_t prevRemoteTx = neighbourData->remoteTx;

      // Local values
      uint64_t localTx = ctx->localTx;
      uint64_t prevLocalRx = neighbourData->localRx;

      // Calculations
      uint32_t remoteReply = remoteTx - remoteRx;
      double clockCorrection = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);
      uint32_t localReply = remoteReply * clockCorrection;

      uint32_t localRound = localRx - localTx;
      neighbourData->tof = (localRound - localReply) / 2;

      // DEBUG
      remoteReply_db = remoteReply;
      localReply_db = localReply;
      localRound_db = localRound;
      tof_db = neighbourData->tof;
      dctCount_db = dict_count(ctx->dct);
      break;
    }
  }

  // Save the remoteTx, so we can use it to calculate the clockCorrection
  neighbourData->remoteTx = rxPacket->tx;
  // Save the localRx, so we can calculate localReply when responding in the future
  neighbourData->localRx = rxTimestamp.full;
}

// DEBUG (to be removed)
LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_UINT32, remoteReply, &remoteReply_db)
LOG_ADD(LOG_UINT32, localReply, &localReply_db)
LOG_ADD(LOG_UINT32, localRound, &localRound_db)
LOG_ADD(LOG_UINT32, tof, &tof_db)
LOG_ADD(LOG_UINT32, dctCount, &dctCount_db)
LOG_GROUP_STOP(twrSwarm)
