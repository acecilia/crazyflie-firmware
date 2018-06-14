#include "TwrSwarmAlgorithmBlocks.h"
#include "clockCorrectionEngine.h"
#include <stdlib.h>

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

/* Clock correction */
/**********************************/

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

/**********************************/


/* Timestamp truncation */
/**********************************/

#define DW1000_MAXIMUM_COUNT_MASK (uint64_t)( 0xFFFFFFFFFF ) //The maximum timestamp the DW1000 can return (40 bits)

/**********************************/


/* Calculation of when to transmit */
/**********************************/

#define AVERAGE_TX_FREQ 400
#define MAX_TX_FREQ 50 // Maximum tx frequency (we do not need more for a proper ranging)
static uint32_t ticksPerSecond = M2T(1000);

/**********************************/

/**
 Calculates a random delay for next transmission
 */
uint32_t calculateRandomDelayToNextTx(uint32_t averageTxDelay) {
  return averageTxDelay / 2 + rand() % averageTxDelay;
}

/**
 Calculates the average tx delay based on the number of drones around
 */
uint32_t calculateAverageTxDelay(uint8_t numberOfNeighbours) {
  uint16_t freq = AVERAGE_TX_FREQ / (numberOfNeighbours + 1);

  if (freq > MAX_TX_FREQ) {
    freq = MAX_TX_FREQ;
  }

  return ticksPerSecond / freq;
}

/**
 Generates a random id, to be used on the packets
 */
locoId_t generateId() {
  return (locoId_t)rand();
}

/**
 Generates a random id which is not found in the provided packet
 */
locoId_t generateIdNotInPacket(lpsSwarmPacket_t* packet) {
  locoId_t cantidateId = generateId();

  // Makes sure the cantidateId is not the source of the packet
  while(packet->sourceId == cantidateId) {
    cantidateId = generateId();
  }

  // Makes sure the cantidateId is not any of the destination ids on the packet
  for(int i = 0; i < packet->payloadLength; i++) {
    if (packet->payload[i].id == cantidateId) {
      return generateIdNotInPacket(packet);
    }
  }

  return cantidateId;
}

/**
 Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0, and round the result up
 */
void adjustTxRxTime(dwTime_t *time) {
  uint64_t mask = (1 << 9) - 1;
  time->full = (time->full & ~mask) + (1 << 9);
}

dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev) {
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // And some extra
  transmitTime.full += TDMA_EXTRA_LENGTH;

  adjustTxRxTime(&transmitTime);

  // Wrap around if needed
  transmitTime.full &= DW1000_MAXIMUM_COUNT_MASK;

  return transmitTime;
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
    data->clockCorrectionStorage.clockCorrection = 1;
    data->clockCorrectionStorage.clockCorrectionBucket = 0;
    data->tof = 0;

    dict_insert_result insert_result = dict_insert(dct, key);
    *insert_result.datum_ptr = data;
    return data;
  }
}

void setTxData(lpsSwarmPacket_t* txPacket, dict* dct, locoId_t sourceId) {
  // Packet creation
  uint8_t payloadLength = dict_count(dct);
  if (payloadLength >)

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
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, lpsSwarmPacket_t* rxPacket, dict* dct, uint64_t lastKnownLocalTxTimestamp) {
  dwTime_t rxTimestamp = { .full = 0 };
  dwGetReceiveTimestamp(dev, &rxTimestamp);
  neighbourData_t* neighbourData = getDataForNeighbour(dct, rxPacket->sourceId);

  // Timestamp remote values
  const uint64_t remoteTx = rxPacket->tx;

  // Timestamp local values
  const uint64_t localRx = rxTimestamp.full;

  for(int i = 0; i < rxPacket->payloadLength; i++) {
    if (rxPacket->payload[i].id == localId) { // To be executed only once

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      debug.succededRangingPerSec++;
#endif

      // Timestamp remote values
      const uint64_t remoteRx = rxPacket->payload[i].time;
      const uint64_t prevRemoteTx = neighbourData->remoteTx;

      // Timestamp local values
      const uint64_t localTx = lastKnownLocalTxTimestamp;
      const uint64_t prevLocalRx = neighbourData->localRx;

      // Calculations
      const uint32_t remoteReply = (uint32_t)(remoteTx - remoteRx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around

      const double clockCorrectionCandidate = clockCorrectionEngine.calculateClockCorrection(localRx, prevLocalRx, remoteTx, prevRemoteTx, 0xFFFFFFFFFF /* 40 bits */);
      clockCorrectionEngine.updateClockCorrection(&neighbourData->clockCorrectionStorage, clockCorrectionCandidate);
      const double clockCorrection = clockCorrectionEngine.getClockCorrection(&neighbourData->clockCorrectionStorage);

      const uint32_t localReply = (uint32_t)(remoteReply * clockCorrection);
      const uint32_t localRound = (uint32_t)(localRx - localTx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around

      // Verify the obtained results are correct
      neighbourData->tof = (localRound - localReply) / 2;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      if (localReply > localRound) {
        debug.measurementFailure++;

        debug.localRx = localRx;
        debug.localTx = localTx;
        debug.remoteRx = remoteRx;
        debug.remoteTx = remoteTx;

        debug.remoteReply = remoteReply;
        debug.localReply = localReply;
        debug.localRound = localRound;
      }

      debug.clockCorrectionCandidate = clockCorrectionCandidate * 10000;
      debug.clockCorrection = clockCorrection * 10000;

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
