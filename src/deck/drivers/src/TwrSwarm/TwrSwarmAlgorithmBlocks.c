#include "TwrSwarmAlgorithmBlocks.h"
#include "clockCorrectionEngine.h"
#include "estimatorKalmanEngine.h"
#include <stdlib.h>
#include "limits.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "task.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#include "debug.h"
#endif

// TODO: remove and do it smarter
#define ANTENNA_OFFSET_METERS 154.6
#define ANTENNA_DELAY (uint16_t)((ANTENNA_OFFSET_METERS * LOCODECK_TS_FREQ) / SPEED_OF_LIGHT)
#define ANTENNA_ACCEPTED_NOISE_ERROR 100
uint16_t antennaDelay = ANTENNA_DELAY;

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

static estimatorKalmanStorage_t storage[2] = {{ .isInit = false },{ .isInit = false }};

/**
 Calculate the distance associated with a tof
 */
float calculateDistance(const uint16_t tof) {
  return SPEED_OF_LIGHT * (tof / LOCODECK_TS_FREQ);
}

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
  for(unsigned int i = 0; i < packet->header.payloadLength; i++) {
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

tofData_t* findTof(tofData_t tofStorage[], const locoId_t id1, const locoId_t id2, const bool insertIfNotFound) {
  unsigned int indexToInitialize = 0;
  // TODO: lifetime span of the tof data
  // uint32_t oldestUpdateTime = xTaskGetTickCount();

  locoIdx2_t id = getHashFromIds(id1, id2);

  for(unsigned int i = 0; i < TOF_STORAGE_CAPACITY; i++) {
    if(tofStorage[i].isInitialized && tofStorage[i].id == id) {
      return &tofStorage[i];
    } else if(insertIfNotFound) {
      if(!tofStorage[i].isInitialized) {
        indexToInitialize = i;
      } else if(/*anchorStorage[i].lastUpdateTime < oldestUpdateTime*/ false) {
        // oldestUpdateTime = anchorStorage[i].lastUpdateTime;
        indexToInitialize = i;
      }
    }
  }

  if(insertIfNotFound) {
    tofStorage[indexToInitialize].isInitialized = true;
    return &tofStorage[indexToInitialize];
  } else {
    return NULL;
  }
}

/**
 Count the number of elements inside the tof storage
 */
unsigned int countTof(tofData_t storage[]) {
  unsigned int count = 0;

  for(unsigned int i = 0; i < TOF_STORAGE_CAPACITY; i++) {
    if(storage[i].isInitialized) {
      count += 1;
    }
  }

  return count;
}

static bool set = false;

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

      if (!set) {
        data->estimatorKalmanStorage = &storage[0];
        set = true;
      } else {
        data->estimatorKalmanStorage = &storage[1];
      }

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

void setTxData(lpsSwarmPacket_t* txPacket, locoId_t sourceId, uint8_t* nextTxSeqNr, dict* neighbourDct, tofData_t tofStorage[]) {
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
      tofData_t* tofData = findTof(tofStorage, sourceId, destinationId, false);
      payload_t pair = {
        .id = destinationId,
        .tx = neighbourData->remoteTx,
        .rx = neighbourData->localRx,
        .tof = tofData != NULL ? tofData->tof : 0
      };
      payload[i] = pair;
      dict_itor_next(itor);
    }

    dict_itor_free(itor);
  }
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, dict* neighboursDct, tofData_t tofStorage[]) {
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

  for(unsigned int i = 0; i < rxPacket->header.payloadLength; i++) {
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

      // TODO: see how can we reduce the error for antenna delay calculation
      // Calculate tof
      uint16_t tofWithAntennaDelay = (uint16_t)((localRound - localReply) / 2);
      if(tofWithAntennaDelay > (ANTENNA_DELAY - ANTENNA_ACCEPTED_NOISE_ERROR) && antennaDelay > tofWithAntennaDelay) {
        // antennaDelay = tofWithAntennaDelay;
      }
      uint16_t tof = tofWithAntennaDelay - antennaDelay;

      // Store tof
      if(tof < 2000) {
        tofData_t* tofData = findTof(tofStorage, localId, remoteId, true);
        tofData->tof = tof;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
        debug.tof = tofData->tof;
        debug.distance = calculateDistance(tofData->tof);
#endif
      }

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      if (localReply > localRound) {
        debug.measurementFailure++;

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
#endif
    } else {
      tofData_t* tofData = findTof(tofStorage, remoteId, payload[i].id, true);
      tofData->tof = (uint16_t)(payload[i].tof * clockCorrection);
    }
  }

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.clockCorrectionCandidate = (uint32_t)(clockCorrectionCandidate * 1000000000);
  debug.clockCorrection = (uint32_t)(clockCorrection * 1000000000);

  debug.dctCount = countTof(tofStorage);
#endif

  // Save the remoteTx, so we can use it to calculate the clockCorrection
  neighbourData->remoteTx = remoteTx;
  // Save the localRx, so we can calculate localReply when responding in the future
  neighbourData->localRx = localRx;

  updatePositionOf(remoteId, neighbourData, neighboursDct, tofStorage);
  updateOwnPosition(localId, remoteId, neighbourData, neighboursDct, tofStorage);

  // HACK for simulating 2D
  tofMeasurement_t tofData;
  tofData.timestamp = xTaskGetTickCount();
  tofData.distance = 0;
  tofData.stdDev = 0;
  estimatorKalmanEnqueueTOF(&tofData);

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  if(neighbourData->estimatorKalmanStorage->isInit) {
    point_t pos;
    estimatorKalmanEngine.getPosition(neighbourData->estimatorKalmanStorage, &pos);
    debug.localRx = pos.x;
  }
  estimatorKalmanGetEstimatedPos(&debug.position);
#endif
}

void updatePositionOf(locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, tofData_t tofStorage[]) {
  distanceMeasurement_t distances[8];
  uint8_t distancesIndex = 0;

  uint8_t neighbours = dict_count(neighboursDct);
  dict_itor *itor = dict_itor_new(neighboursDct);
  dict_itor_first(itor);
  for (unsigned int i = 0; i < neighbours; i++) {
    locoId_t neighbourId = *(locoId_t*)dict_itor_key(itor);

    if (remoteId != neighbourId) {
      tofData_t* tofData = findTof(tofStorage, remoteId, neighbourId, false);

      if (tofData != NULL) {
        neighbourData_t* neighbourData = (neighbourData_t*)*dict_itor_datum(itor);
        if(neighbourData->estimatorKalmanStorage->isInit) {
          point_t positionData;
          estimatorKalmanEngine.getPosition(neighbourData->estimatorKalmanStorage, &positionData);

          distances[distancesIndex].x = positionData.x;
          distances[distancesIndex].y = positionData.y;
          distances[distancesIndex].z = positionData.z;
          distances[distancesIndex].distance = calculateDistance(tofData->tof);
          distances[distancesIndex].stdDev = 0.01;
          distancesIndex++;
        }
      }
    }
    dict_itor_next(itor);
  }
  dict_itor_free(itor);

  if(!neighbourData->estimatorKalmanStorage->isInit) {
    const velocity_t initialVelocity = { .x = 0, .y = 0, .z = 0 };
    if(neighbours == 1) {
      // Set the first discovered copter as the (0, 0, 0) point of the about-to-be-defined coordinate system
      point_t initialPosition = { .x = 0, .y = 0, .z = 0 };
      estimatorKalmanEngine.init(neighbourData->estimatorKalmanStorage, initialPosition, initialVelocity);
    } else if(neighbours == 2 && distancesIndex == 1) {
      // Set the second discovered copter in the positive part of the x axis
      float x = distances[0].x + distances[0].distance;
      point_t initialPosition = { .x = x, .y = 0, .z = 0 };
      estimatorKalmanEngine.init(neighbourData->estimatorKalmanStorage, initialPosition, initialVelocity);
    } else if(neighbours == 3 && distancesIndex == 2) {
      // Set the third discovered copter in the possitive part of the y axis
      // TODO: trilaterate the initial position related to the other drones, instead of giving fixed values
      point_t initialPosition = { .x = 0, .y = 1, .z = 0 };
      estimatorKalmanEngine.init(neighbourData->estimatorKalmanStorage, initialPosition, initialVelocity);
    } else if(neighbours == 4 && distancesIndex == 3) {
      // Set the fourth discovered copter in the possitive part of the z axis
      // TODO: trilaterate the initial position related to the other drones, instead of giving fixed values
      point_t initialPosition = { .x = 0, .y = 0, .z = 1 };
      estimatorKalmanEngine.init(neighbourData->estimatorKalmanStorage, initialPosition, initialVelocity);
      // TODO: set coordinate system already created
    } else if(false /* coordinate system already created */) {
      // TODO: trilaterate the initial position related to the other drones, instead of giving fixed values
      point_t initialPosition = { .x = 0, .y = 0, .z = 0 };
      estimatorKalmanEngine.init(neighbourData->estimatorKalmanStorage, initialPosition, initialVelocity);
    } else {
      // There are not enough distances to calculate the position
      DEBUG_PRINT("Neighbours: %d | distances: %d\n", neighbours, distancesIndex);
      // configASSERT(false);
      // TODO: next cases
      return;
    }
  }

  if(true /* is building the coordinate system*/) {
    if(neighbours == 2) {
      // Simulate 1D
      positionMeasurement_t position = { .x = NAN, .y = 0, .z = 0, .stdDev = 0 };
      estimatorKalmanEngine.enqueuePosition(neighbourData->estimatorKalmanStorage, position);
    } else if (neighbours == 3) {
      // Simulate 2D
      positionMeasurement_t position = { .x = NAN, .y = NAN, .z = 0, .stdDev = 0 };
      estimatorKalmanEngine.enqueuePosition(neighbourData->estimatorKalmanStorage, position);
    }
  }

  for(unsigned int i = 0; i < distancesIndex; i++) {
    estimatorKalmanEngine.enqueueDistance(neighbourData->estimatorKalmanStorage, distances[i]);
  }

  estimatorKalmanEngine.update(neighbourData->estimatorKalmanStorage);
}

void updateOwnPosition(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, tofData_t tofStorage[]) {
  if(neighbourData->estimatorKalmanStorage->isInit) {
    point_t remotePosition;
    estimatorKalmanEngine.getPosition(neighbourData->estimatorKalmanStorage, &remotePosition);

    tofData_t* tofData = findTof(tofStorage, localId, remoteId, false);
    if(tofData != NULL) {
      distanceMeasurement_t dist = {
        .distance = calculateDistance(tofData->tof),
        .x = remotePosition.x,
        .y = remotePosition.y,
        .z = remotePosition.z,
        .stdDev = 0.25
      };
      estimatorKalmanEnqueueDistance(&dist);
    }
  }
}
