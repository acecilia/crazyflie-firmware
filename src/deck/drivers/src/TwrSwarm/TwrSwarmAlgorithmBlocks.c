#include "TwrSwarmAlgorithmBlocks.h"
#include "clockCorrectionEngine.h"
#include <stdlib.h>
#include "limits.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

// #pragma GCC diagnostic warning "-Wconversion"

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

#define DW1000_MAXIMUM_COUNT_MASK (uint64_t)(0xFFFFFFFFFF) //The maximum timestamp the DW1000 can return (40 bits)

/**********************************/


/* Calculation of when to transmit */
/**********************************/

#define AVERAGE_TX_FREQ 400
#define MAX_TX_FREQ 100 // Maximum tx frequency (we do not need more for a proper ranging)
static uint32_t ticksPerSecond = M2T(1000);

/**********************************/

/**
 Verify if a packed sequence number is valid
 */
bool verifySeqNr(const uint8_t seqNr, const uint8_t expectedSeqNr) {
  // If the sequence number was from 0 to 5 units less than the expectedSeqNr, we consider that packet a delayed reflexion
  return ((uint8_t)(seqNr - expectedSeqNr)) <= UCHAR_MAX - 5;
}


/**
 Calculates a unique id from two loco ids
 */
locoIdx2_t getHashFromIds(const locoId_t id1, const locoId_t id2) {
  locoId_t leftPart;
  locoId_t rightPart;

  if (id1 >= id2) {
    leftPart = id1;
    rightPart = id2;
  } else {
    leftPart = id2;
    rightPart = id1;
  }

  return (locoIdx2_t)((leftPart << (sizeof(locoId_t) * CHAR_BIT)) | rightPart);
}

/**
 Calculates a random delay for next transmission
 */
uint32_t calculateRandomDelayToNextTx(uint32_t averageTxDelay) {
  return averageTxDelay / 2 + (uint32_t)rand() % averageTxDelay;
}

/**
 Calculates the average tx delay based on the number of drones around
 */
uint32_t calculateAverageTxDelay(uint8_t numberOfNeighbours) {
  uint16_t freq = (uint16_t)(AVERAGE_TX_FREQ / (numberOfNeighbours + 1));

  if (freq > MAX_TX_FREQ) {
    freq = MAX_TX_FREQ;
  }

  return ticksPerSecond / freq;
}

/**
 Makes sure the seed for rand is set before using it
 */
static bool randIsInit = false;

/**
 Set a random seed for the rand function
 */
void initRandomizationEngine(dwDevice_t *dev) {
  dwTime_t now = { .full = 0 };
  dwGetSystemTimestamp(dev, &now);

  srand(now.low32);
  randIsInit = true;
}

/**
 Generates a random id, to be used on the packets
 */
locoId_t generateId() {
  ASSERT(randIsInit);
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

  // Makes sure the candidateId is not the id of any of the known drones
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

tofData_t* getTofDataBetween(dict* dct, const locoId_t id1, const locoId_t id2, const bool insertIfNotFound) {
  locoIdx2_t id = getHashFromIds(id1, id2);

  void** search_result = dict_search(dct, &id);
  if (search_result) {
    return (tofData_t *)*search_result;
  } else {
    if (insertIfNotFound) {
      locoIdx2_t* key = pvPortMalloc(sizeof(locoIdx2_t));
      *key = id;

      tofData_t* data = pvPortMalloc(sizeof(tofData_t));
      data->tof = 0;

      dict_insert_result insert_result = dict_insert(dct, key);
      *insert_result.datum_ptr = data;
      return data;
    } else {
      return NULL;
    }
  }
}

neighbourData_t* getDataForNeighbour(dict* dct, const locoId_t id, const bool insertIfNotFound) {
  void** search_result = dict_search(dct, &id);
  if (search_result) {
    return (neighbourData_t *)*search_result;
  } else {
    if (insertIfNotFound) {
      locoId_t* key = pvPortMalloc(sizeof(locoId_t));
      *key = id;

      neighbourData_t* data = pvPortMalloc(sizeof(neighbourData_t));
      data->localRx = 0;
      data->remoteTx = 0;
      data->clockCorrectionStorage.clockCorrection = 1;
      data->clockCorrectionStorage.clockCorrectionBucket = 0;
      data->expectedSeqNr = 0;
      data->position = (point_t){
        .timestamp = 0,
        .x = 0,
        .y = 0,
        .z = 0
      };

      dict_insert_result insert_result = dict_insert(dct, key);
      *insert_result.datum_ptr = data;
      return data;
    } else {
      return NULL;
    }
  }
}

unsigned int calculatePacketSize(lpsSwarmPacket_t* packet) {
  return sizeof(lpsSwarmPacketHeader_t) + packet->header.payloadLength * sizeof(payload_t);
}

void setTxData(lpsSwarmPacket_t* txPacket, locoId_t sourceId, uint8_t* nextTxSeqNr, dict* neighbourDct, dict* tofDict) {
  uint8_t payloadLength = dict_count(neighbourDct);

  txPacket->header.tx = 0; // This will be filled after setting the data of the txPacket, and inmediatelly before starting the transmission. For now, we zero it
  txPacket->header.sourceId = sourceId;
  txPacket->header.seqNr = *nextTxSeqNr; *nextTxSeqNr += 1;
  txPacket->header.payloadLength = payloadLength;

  payload_t* payload = (payload_t*)&(txPacket->payload);

  if (payloadLength > 0) {
    // Get data from the dict and into the txPacket array
    dict_itor *itor = dict_itor_new(neighbourDct);
    dict_itor_first(itor);
    for (unsigned int i = 0; i < payloadLength; i++) {
      locoId_t destinationId = *(locoId_t*)dict_itor_key(itor);
      neighbourData_t* neighbourData = (neighbourData_t*)*dict_itor_datum(itor);
      tofData_t* tofData = getTofDataBetween(tofDict, sourceId, destinationId, true);
      payload_t pair = {
        .id = destinationId,
        .tx = neighbourData->remoteTx,
        .rx = neighbourData->localRx,
        .tof = tofData->tof
      };
      payload[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, dict* neighboursDct, dict* tofDct) {
  // Get neighbour data
  locoId_t remoteId = rxPacket->header.sourceId;
  neighbourData_t* neighbourData = getDataForNeighbour(neighboursDct, remoteId, true);

  // Check sequence number
  uint8_t seqNr = rxPacket->header.seqNr;
  if (verifySeqNr(seqNr, neighbourData->expectedSeqNr) == false) {
    // Received packet is a delayed reflection
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
    debug.reflections++;
#endif
    return;
  } else {
    neighbourData->expectedSeqNr = (uint8_t)(seqNr + 1);
  }

  // Get rx timestamp
  dwTime_t rxTimestamp = { .full = 0 };
  dwGetReceiveTimestamp(dev, &rxTimestamp);

  // Timestamp remote values
  const uint64_t prevRemoteTx = neighbourData->remoteTx;
  const uint64_t remoteTx = rxPacket->header.tx;

  // Timestamp local values
  const uint64_t prevLocalRx = neighbourData->localRx;
  const uint64_t localRx = rxTimestamp.full;

  // Calculate clock correction
  const double clockCorrectionCandidate = clockCorrectionEngine.calculateClockCorrection(localRx, prevLocalRx, remoteTx, prevRemoteTx, 0xFFFFFFFFFF /* 40 bits */);
  bool clockCorrectionCandidateAccepted = clockCorrectionEngine.updateClockCorrection(&neighbourData->clockCorrectionStorage, clockCorrectionCandidate);
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.clockUpdated++;
  if (!clockCorrectionCandidateAccepted) {
    debug.clockNotAccepted++;
  }
#endif
  const double clockCorrection = clockCorrectionEngine.getClockCorrection(&neighbourData->clockCorrectionStorage);

  // Cast payload, for easier access
  payload_t* payload = (payload_t*)&(rxPacket->payload);

  for(int i = 0; i < rxPacket->header.payloadLength; i++) {
    if (payload[i].id == localId) {
      // TODO: Andres. See what can we use the transmitted tof for

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      debug.succededRangingPerSec++;
#endif

      // Timestamp remote values
      const uint64_t remoteRx = payload[i].rx;

      // Timestamp local values
      const uint64_t localTx = payload[i].tx;

      // Calculations
      const uint32_t remoteReply = (uint32_t)(remoteTx - remoteRx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around
      const uint32_t localReply = (uint32_t)(remoteReply * clockCorrection);
      const uint32_t localRound = (uint32_t)(localRx - localTx); // Casting uint64_t to uint32_t removes the effect of the clock wrapping around

      tofData_t* tofData = getTofDataBetween(tofDct, localId, remoteId, true);
      tofData->tof = (uint16_t)((localRound - localReply) / 2);

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      if (localReply > localRound) {
        debug.measurementFailure++;

        debug.localRx = localRx;
        debug.localTx = localTx;
        debug.remoteRx = remoteRx;
        debug.remoteTx = remoteTx;
      }

      /*
      uint32_t localReply2 = (uint32_t)(remoteReply * clockCorrection);
      if (!clockCorrectionCandidateAccepted) {
        const uint64_t tickCount_in_cl_x = (remoteTx - prevRemoteTx) & 0xFFFFFFFFFF;
        const int32_t replyCorrection = (int32_t)((clockCorrectionCandidate - clockCorrection) * tickCount_in_cl_x);
        localReply2 += replyCorrection;
      }*/
      const uint32_t localReply2 = (uint32_t)(remoteReply * clockCorrectionCandidate);
      debug.auxiliaryValue = (localRound - localReply2) / 2;

      debug.localRound = localRound;
      debug.localReply = localReply;
      debug.remoteReply = remoteReply;

      debug.tof = tofData->tof;
#endif
    } else {
      tofData_t* tofData = getTofDataBetween(tofDct, remoteId, payload[i].id, true);
      tofData->tof = (uint16_t)(payload[i].tof * clockCorrection);
    }
  }

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.clockCorrectionCandidate = (uint32_t)(clockCorrectionCandidate * 1000000000);
  debug.clockCorrection = (uint32_t)(clockCorrection * 1000000000);

  debug.dctCount = dict_count(tofDct);
#endif

  // Save the remoteTx, so we can use it to calculate the clockCorrection
  neighbourData->remoteTx = remoteTx;
  // Save the localRx, so we can calculate localReply when responding in the future
  neighbourData->localRx = localRx;

  updatePositionOf(localId, remoteId, neighbourData, neighboursDct, tofDct);
  updateOwnPosition(localId, remoteId, neighbourData, neighboursDct, tofDct);
}

void updatePositionOf(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, dict* tofDct) {
  distanceMeasurement_t distances[8];
  uint8_t distancesIndex = 0;

  uint8_t neighbours = dict_count(neighboursDct);
  dict_itor *itor = dict_itor_new(neighboursDct);
  dict_itor_first(itor);
  for (unsigned int i = 0; i < neighbours; i++) {
    locoId_t neighbourId = *(locoId_t*)dict_itor_key(itor);

    if (remoteId != neighbourId && neighbourId != localId) {
      tofData_t* tofData = getTofDataBetween(tofDct, remoteId, neighbourId, false);

      if (tofData != NULL) {
        point_t* positionData = &((neighbourData_t*)*dict_itor_datum(itor))->position;

        distanceMeasurement_t* distanceContainer = &distances[distancesIndex];
        distanceContainer->x = positionData->x;
        distanceContainer->y = positionData->y;
        distanceContainer->z = positionData->z;
        distanceContainer->distance = tofData->tof;

        distancesIndex++;
      }
    }
    dict_itor_next(itor);
  }
  dict_itor_free(itor);

  // Distances are ready: set position
  point_t* position = &neighbourData->position;
  if(distancesIndex == 0) {
    // Set the first discovered copter as the (0, 0, 0) point of the about-to-be-defined coordinate system
    position->x = 0;
    position->y = 0;
    position->z = 0;
  } else if(distancesIndex == 1) {
    // Set the second discovered copter in the X axis, following the first copter position
    if (position->x >= distances[0].x) {
      position->x = distances[0].x + distances[0].distance;
    } else {
      position->x = distances[0].x - distances[0].distance;
    }
    position->y = distances[0].y;
    position->z = distances[0].z;
  }
  // TODO: next cases
}

void updateOwnPosition(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, dict* tofDct) {
  point_t* remotePosition = &neighbourData->position;
  tofData_t* tofData = getTofDataBetween(tofDct, localId, remoteId, false);

  if(tofData != NULL) {
    distanceMeasurement_t dist;
    dist.distance = tofData->tof;
    dist.x = remotePosition->x;
    dist.y = remotePosition->y;
    dist.z = remotePosition->z;
    dist.stdDev = 0.25;
    estimatorKalmanEnqueueDistance(&dist);
  }
}
