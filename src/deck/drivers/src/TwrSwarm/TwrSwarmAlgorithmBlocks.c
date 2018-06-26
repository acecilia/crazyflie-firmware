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
locoId_t generateIdNotIn(lpsSwarmPacket_t* packet, dict* dct) {
  locoId_t cantidateId = generateId();

  // Makes sure the cantidateId is not the source of the packet
  if(packet->header.sourceId == cantidateId) {
    return generateIdNotIn(packet, dct);
  }

  // Makes sure the cantidateId is not any of the destination ids on the packet
  for(int i = 0; i < packet->header.payloadLength; i++) {
    payload_t* payload = (payload_t*)&(packet->payload);
    if (payload[i].id == cantidateId) {
      return generateIdNotIn(packet, dct);
    }
  }

  // Makes sure the cantidateId is not the id of any of the known drones
  void** search_result = dict_search(dct, &cantidateId);
  if (search_result) {
    return generateIdNotIn(packet, dct);
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

unsigned int calculatePacketSize(lpsSwarmPacket_t* packet) {
  return sizeof(lpsSwarmPacketHeader_t) + packet->header.payloadLength * sizeof(payload_t);
}

void setTxData(lpsSwarmPacket_t* txPacket, dict* dct, locoId_t sourceId) {
  uint8_t payloadLength = dict_count(dct);

  txPacket->header.tx = 0; // This will be filled after setting the data of the txPacket, and inmediatelly before starting the transmission. For now, we zero it
  txPacket->header.sourceId = sourceId;
  txPacket->header.payloadLength = payloadLength;

  payload_t* payload = (payload_t*)&(txPacket->payload);

  if (payloadLength > 0) {
    // Get data from the dict and into the txPacket array
    dict_itor *itor = dict_itor_new(dct);
    dict_itor_first(itor);
    for (unsigned int i = 0; i < payloadLength; i++) {
      locoId_t key = *(locoId_t*)dict_itor_key(itor);
      neighbourData_t* data = (neighbourData_t*)*dict_itor_datum(itor);
      payload_t pair = {
        .id = key,
        .tx = data->remoteTx,
        .rx = data->localRx,
      };
      payload[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, dict* dct) {
  dwTime_t rxTimestamp = { .full = 0 };
  dwGetReceiveTimestamp(dev, &rxTimestamp);

  neighbourData_t* neighbourData = getDataForNeighbour(dct, rxPacket->header.sourceId);
  payload_t* payload = (payload_t*)&(rxPacket->payload);

  // Timestamp remote values
  const uint64_t prevRemoteTx = neighbourData->remoteTx;
  const uint64_t remoteTx = rxPacket->header.tx;

  // Timestamp local values
  const uint64_t prevLocalRx = neighbourData->localRx;
  const uint64_t localRx = rxTimestamp.full;

  // Calculate clock correction
  const double clockCorrectionCandidate = clockCorrectionEngine.calculateClockCorrection(localRx, prevLocalRx, remoteTx, prevRemoteTx, 0xFFFFFFFFFF /* 40 bits */);
  clockCorrectionEngine.updateClockCorrection(&neighbourData->clockCorrectionStorage, clockCorrectionCandidate);

  for(int i = 0; i < rxPacket->header.payloadLength; i++) {
    if (payload[i].id == localId) { // To be executed only once

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      debug.succededRangingPerSec++;
#endif

      // Timestamp remote values
      const uint64_t remoteRx = payload[i].rx;

      // Timestamp local values
      const uint64_t localTx = payload[i].tx;

      // Calculations
      const double clockCorrection = clockCorrectionEngine.getClockCorrection(&neighbourData->clockCorrectionStorage);
      const uint32_t remoteReply = (uint32_t)(remoteTx - remoteRx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around
      const uint32_t localReply = (uint32_t)(remoteReply * clockCorrection);
      const uint32_t localRound = (uint32_t)(localRx - localTx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around

      neighbourData->tof = (localRound - localReply) / 2;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      /*if (localReply > localRound) {
        debug.measurementFailure++;

        debug.localRx = localRx;
        debug.localTx = localTx;
        debug.remoteRx = remoteRx;
        debug.remoteTx = remoteTx;

        debug.remoteReply = remoteReply;
        debug.localReply = localReply;
        debug.localRound = localRound;
      }*/

      debug.auxiliaryValue = (localRound - remoteReply) / 2;

      debug.localRound = localRound;
      debug.localReply = localReply;
      debug.remoteReply = remoteReply;

      debug.clockCorrectionCandidate = (uint32_t)(clockCorrectionCandidate * 1000000000);
      debug.clockCorrection = (uint32_t)(clockCorrection * 1000000000);

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
