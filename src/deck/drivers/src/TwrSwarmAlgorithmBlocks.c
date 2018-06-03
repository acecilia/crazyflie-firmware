#include "TwrSwarmAlgorithmBlocks.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

/**
 The DW1000 has a 40 bits register to store the timestamp values. When the timestamp is higher than what is possible to store in those 40 bits, the count wraps around, despite a uint64_t value having enough bits to represent the number. This function reverses the wrap around.
 */
static uint64_t fixWrapAroundIfNeeded(uint64_t minimumValue, uint64_t valueToFix) {
  if (minimumValue >= valueToFix) {
    uint64_t maximumDw1000Count = 0xffffffffff; //The maximum timestamp the DW1000 can return (40 bits)
    return maximumDw1000Count + valueToFix;
  } else {
    return valueToFix;
  }
}

locoId_t getId(locoAddress_t address) {
  return address & 0xff;
}

// Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0, and round the result up
void adjustTxRxTime(dwTime_t *time) {
  time->full = (time->full & ~((1 << 9) - 1)) + (1 << 9);
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

  uint32_t tickCountRemote = fixWrapAroundIfNeeded(prevRemoteTx, remoteTx) - prevRemoteTx;
  uint32_t tickCountLocal = fixWrapAroundIfNeeded(prevLocalRx, localRx) - prevLocalRx;

  if (tickCountRemote > 0) {
    result = (double)tickCountLocal / (double)tickCountRemote;
  }

  return result;
}

neighbourData_t* getDataForNeighbour(dict* dct, locoId_t id) {
  void** search_result = dict_search(dct, &id);
  if (search_result) {
    return (neighbourData_t *)*search_result;
  } else {
    locoAddress_t* key = pvPortMalloc(sizeof(locoId_t));
    *key = id;

    neighbourData_t* data = pvPortMalloc(sizeof(neighbourData_t));
    data->localRx = 0;
    data->remoteTx = 0;
    data->tof = 0;

    dict_insert_result insert_result = dict_insert(dct, key);
    *insert_result.datum_ptr = data;
    return data;
  }
}

unsigned int allocAndFillTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoId_t sourceId) {
  // Packet creation
  unsigned int payloadLength = dict_count(dct);
  unsigned int txPacketLength = sizeof(lpsSwarmPacket_t) + payloadLength * sizeof(payload_t);

  *txPacketPointer = pvPortMalloc(txPacketLength);

  lpsSwarmPacket_t* txPacket = *txPacketPointer; // Declared for convenience and cleaner code
  txPacket->tx = 0; // This will be filled after allocating and filling the txPacket, and inmediatelly before starting the transmission. For now, we zero it.
  txPacket->sourceId = sourceId;
  txPacket->payloadLength = payloadLength;

  if (payloadLength > 0) {
    // Get data from the dict and into the txPacket array
    dict_itor *itor = dict_itor_new(dct);
    dict_itor_first(itor);
    for (unsigned int i = 0; i < payloadLength; i++) {
      locoId_t key = *(locoId_t*)dict_itor_key(itor);
      neighbourData_t* data = (neighbourData_t*)*dict_itor_datum(itor);
      payload_t pair = {
        .id = key,
        .time = data->localRx
      };

      txPacket->payload[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }

  return txPacketLength;
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, lpsSwarmPacket_t* rxPacket, dict* dct, uint64_t lastKnownLocalTxTimestamp) {
  dwTime_t rxTimestamp = { .full = 0 };
  dwGetReceiveTimestamp(dev, &rxTimestamp);
  neighbourData_t* neighbourData = getDataForNeighbour(dct, rxPacket->sourceId);

  // Timestamp remote values
  uint64_t remoteTx = rxPacket->tx;

  // Timestamp local values
  uint64_t localRx = rxTimestamp.full;

  for(int i = 0; i < rxPacket->payloadLength; i++) {
    if (rxPacket->payload[i].id == localId) { // To be executed only once

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      debug.succededRangingPerSec++;
#endif

      // Timestamp remote values
      uint64_t remoteRx = rxPacket->payload[i].time;
      uint64_t prevRemoteTx = neighbourData->remoteTx;

      // Timestamp local values
      uint64_t localTx = lastKnownLocalTxTimestamp;
      uint64_t prevLocalRx = neighbourData->localRx;

      // Calculations
      uint32_t remoteReply = fixWrapAroundIfNeeded(remoteRx, remoteTx) - remoteRx;
      double clockCorrection = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);
      uint32_t localReply = remoteReply * clockCorrection;
      uint32_t localRound = fixWrapAroundIfNeeded(localTx, localRx) - localTx;

      // Verify the obtained results are correct
      neighbourData->tof = (localRound - localReply) / 2;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      if (localReply > localRound) {
        debug.measurementFailure++;
      }

      debug.remoteReply = remoteReply;
      debug.remoteRx = remoteRx;
      debug.remoteTx = remoteTx;

      debug.localReply = localReply;

      debug.localRound = localRound;
      debug.localRx = localRx;
      debug.localTx = localTx;

      debug.tof = neighbourData->tof;

      debug.dctCount = dict_count(dct);
#endif
      break;
    }
  }

  // Save the remoteTx, so we can use it to calculate the clockCorrection
  neighbourData->remoteTx = remoteTx;
  // Save the localRx, so we can calculate localReply when responding in the future
  neighbourData->localRx = localRx;
}
