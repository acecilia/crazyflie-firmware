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
#endif

// Example of debug print
/*
#include "debug.h"

if(false) { DEBUG_PRINT("PS: %f, %f, %f\n", (double)neighbourData->estimator.P[STATE_X][STATE_X], (double)neighbourData->estimator.P[STATE_Y][STATE_Y], (double)neighbourData->estimator.P[STATE_Z][STATE_Z]); }
 */

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

#define DW1000_MAXIMUM_COUNT_MASK (uint64_t)(createMask(40)) // The maximum timestamp the DW1000 can return (40 bits)

static inline uint64_t min(const uint64_t left, const uint64_t right) {
  if (left <= right) {
    return left;
  } else {
    return right;
  }
}

uint64_t createMask(const uint8_t numberOfOnes) {
  uint64_t mask = 0;
  for(unsigned int i = 0; i < numberOfOnes; i++) {
    mask |= ((uint64_t) 1 << i);
  }
  return mask;
}

#define PACKET_TIMESTAMP_MASK (uint64_t)(min(DW1000_MAXIMUM_COUNT_MASK, createMask(sizeof(packetTimestamp_t) * CHAR_BIT))) // The mask to use when working with timestamps that are coming from a transmission or are going to be transmitted

/**********************************/


/* Calculation of when to transmit */
/**********************************/

#define AVERAGE_TX_FREQ 400
#define MAX_TX_FREQ 100 // Maximum tx frequency (we do not need more for a proper ranging)
#define MIN_TX_FREQ 20 // The TX timestamp in the protocol is only 32 bits (equal to 67 ms) and we want to avoid double wraps of the TX counter. To have some margin set the lowest tx frequency to 20 Hz (= 50 ms)

static uint32_t ticksPerSecond = M2T(1000);

/**********************************/


/* Other constants */
/**********************************/

#define TOF_TO_DISTANCE SPEED_OF_LIGHT / LOCODECK_TS_FREQ

/**********************************/

void initEstimatorKalmanEngine() {
  estimatorKalmanEngine.initializeEngine(xTaskGetTickCount, configTICK_RATE_HZ);
}

/**
 Count the number of elements inside the neighbours storage
 */
unsigned int countNeighbours(neighbourData_t storage[]) {
  unsigned int count = 0;

  for(unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    if(storage[i].isInitialized) {
      count += 1;
    }
  }

  return count;
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

  locoId_t array[2] = {rightPart, leftPart};
  return *(locoIdx2_t*)array;
}

bool hashContainsId(const locoIdx2_t hash, const locoId_t id) {
  locoId_t* array = (locoId_t*)&hash;
  return array[0] == id || array[1] == id;
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
uint32_t calculateAverageTxDelay(neighbourData_t neighboursStorage[]) {
  uint8_t numberOfNeighbours = countNeighbours(neighboursStorage);

  uint16_t freq = (uint16_t)(AVERAGE_TX_FREQ / (numberOfNeighbours + 1));

  if (freq > MAX_TX_FREQ) {
    freq = MAX_TX_FREQ;
  } else if (freq < MIN_TX_FREQ) {
    freq = MIN_TX_FREQ;
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
locoId_t generateIdNotInPacket(neighbourData_t neighboursStorage[], lpsSwarmPacket_t* packet) {
  locoId_t cantidateId = generateId();

  // Makes sure the cantidateId is not the source of the packet
  if(packet->header.sourceId == cantidateId) {
    return generateIdNotInPacket(neighboursStorage, packet);
  }

  // Makes sure the cantidateId is not any of the destination ids on the packet
  for(unsigned int i = 0; i < packet->header.payloadLength; i++) {
    if (packet->payload[i].id == cantidateId) {
      return generateIdNotInPacket(neighboursStorage, packet);
    }
  }

  // Makes sure the candidateId is not the id of any of the known drones
  neighbourData_t* searchResult = findNeighbourData(neighboursStorage, cantidateId, false);
  if (searchResult) {
    return generateIdNotInPacket(neighboursStorage, packet);
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

tofData_t* findTofData(tofData_t storage[], const locoId_t id1, const locoId_t id2, const bool insertIfNotFound) {
  int indexToInitialize = -1;
  locoIdx2_t id = getHashFromIds(id1, id2);

  for(unsigned int i = 0; i < TOF_STORAGE_CAPACITY; i++) {
    if(storage[i].isInitialized && storage[i].id == id) {
      return &storage[i];
    } else if(insertIfNotFound && indexToInitialize < 0 && !storage[i].isInitialized) {
      indexToInitialize = i;
    }
  }

  if(insertIfNotFound) {
    storage[indexToInitialize].isInitialized = true;
    storage[indexToInitialize].id = id;
    return &storage[indexToInitialize];
  } else {
    return NULL;
  }
}

void removeOutdatedData(neighbourData_t neighbourDataStorage[], tofData_t tofDataStorage[]) {
  for(unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    neighbourData_t* neighbourData = &neighbourDataStorage[i];

    if (neighbourData->isInitialized && !neighbourData->IsValid) {
      // Remove the tof associated with the neighbour
      for(unsigned int e = 0; e < TOF_STORAGE_CAPACITY; e++) {
        tofData_t* tofData = &tofDataStorage[e];
        locoIdx2_t hash = tofData->id;
        if (hashContainsId(hash, neighbourData->id)){
          memset(tofData, 0, sizeof(tofData_t));
        }
      }

      // Remove the neighbour
      memset(neighbourData, 0, sizeof(neighbourData_t));
    }

    neighbourData->IsValid = false;
  }
}

neighbourData_t* findNeighbourData(neighbourData_t storage[], const locoId_t id, const bool insertIfNotFound) {
  int indexToInitialize = -1;

  for(unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    if(storage[i].isInitialized && storage[i].id == id) {
      return &storage[i];
    } else if(insertIfNotFound && indexToInitialize < 0 && !storage[i].isInitialized) {
      indexToInitialize = i;
    }
  }

  if(insertIfNotFound) {
    storage[indexToInitialize].isInitialized = true;
    storage[indexToInitialize].id = id;

    // TODO. acecilia. Make a init function in the clockCorrection engine
    storage[indexToInitialize].clockCorrectionStorage = (clockCorrectionStorage_t){
      .clockCorrection = 1,
      .clockCorrectionBucket = 0
    };
    storage[indexToInitialize].expectedSeqNr = 0;
    return &storage[indexToInitialize];
  } else {
    return NULL;
  }
}

unsigned int calculatePacketSize(lpsSwarmPacket_t* packet) {
  return sizeof(lpsSwarmPacketHeader_t) + packet->header.payloadLength * sizeof(payload_t);
}

void setTxData(lpsSwarmPacket_t* txPacket, locoId_t sourceId, uint8_t* nextTxSeqNr, neighbourData_t neighboursStorage[], tofData_t tofStorage[]) {
  txPacket->header.tx = 0; // This will be filled after setting the data of the txPacket, and inmediatelly before starting the transmission. For now, we zero it
  txPacket->header.sourceId = sourceId;
  txPacket->header.seqNr = *nextTxSeqNr; *nextTxSeqNr += 1;

  uint8_t payloadLength = 0;

  // Get data from the dict and into the txPacket array
  for (unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    neighbourData_t* neighbourData = &neighboursStorage[i];
    if(neighbourData->isInitialized) {
      tofData_t* tofData = findTofData(tofStorage, sourceId, neighboursStorage[i].id, false);
      payload_t pair = {
        .id = neighboursStorage[i].id,
        .tx = neighbourData->remoteTx,
        .rx = neighbourData->localRx,
        .tof = tofData != NULL ? tofData->tof : 0
      };
      txPacket->payload[payloadLength] = pair;
      payloadLength += 1;
    }
  }

  txPacket->header.payloadLength = payloadLength;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.sentPacketsPerSec++;
#endif
}

void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, const uint16_t antennaDelay, bool* isBuildingCoordinateSystem, neighbourData_t neighboursStorage[], tofData_t tofStorage[]) {
  // Get neighbour data
  locoId_t neighbourId = rxPacket->header.sourceId;
  neighbourData_t* neighbourData = findNeighbourData(neighboursStorage, neighbourId, true);
  neighbourData->IsValid = true;

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
  const packetTimestamp_t prevRemoteTx = neighbourData->remoteTx;
  const packetTimestamp_t remoteTx = rxPacket->header.tx;

  // Timestamp local values
  const packetTimestamp_t prevLocalRx = neighbourData->localRx;
  const packetTimestamp_t localRx = rxTimestamp.full;

  // Calculate clock correction
  const double clockCorrectionCandidate = clockCorrectionEngine.calculateClockCorrection(localRx, prevLocalRx, remoteTx, prevRemoteTx, PACKET_TIMESTAMP_MASK);
  bool clockCorrectionCandidateAccepted = clockCorrectionEngine.updateClockCorrection(&neighbourData->clockCorrectionStorage, clockCorrectionCandidate);
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.clockUpdated++;
  if (!clockCorrectionCandidateAccepted) {
    debug.clockNotAccepted++;
  }
#endif
  const double clockCorrection = clockCorrectionEngine.getClockCorrection(&neighbourData->clockCorrectionStorage);
  
  for(unsigned int i = 0; i < rxPacket->header.payloadLength; i++) {
    if (rxPacket->payload[i].id == localId) {
      // TODO: acecilia. See what can we use the transmitted tof for

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
      debug.receivedPacketsPerSec++;
#endif
      
      // Timestamp remote values
      const packetTimestamp_t remoteRx = rxPacket->payload[i].rx;

      // Timestamp local values
      const packetTimestamp_t localTx = rxPacket->payload[i].tx;

      // Calculations
      const packetTimestamp_t remoteReply = remoteTx - remoteRx;
      const packetTimestamp_t localReply = (packetTimestamp_t)(remoteReply * clockCorrection);
      const packetTimestamp_t localRound = localRx - localTx;

      // TODO: see how can we reduce the error for antenna delay calculation
      // Calculate tof
      uint16_t tofWithAntennaDelay = (uint16_t)((localRound - localReply) / 2);
      uint16_t tof = tofWithAntennaDelay - antennaDelay;

      // Store tof. Reject outliers
      // acecilia. TODO: use leaky bucket?
      if(tof < 2131) { // 2131 = 10 meters
        tofData_t* tofData = findTofData(tofStorage, localId, neighbourId, true);
        tofData->tof = tof;

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
        debug.tof = tofData->tof;
        debug.distance = tofData->tof * TOF_TO_DISTANCE;
        debug.succededTofCalculationPerSec++;
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
      // In order to accept the tof between the neighbour and a remoteNeighbour, both quadcopters should be known: they should be in the neighbour storage. The neighbour sending the packet is already in the storage, so the remoteNeighbour is the one left to check
      // NOTE: without this check, removing the outdated data of a quadcopter will not fully work: after removal, it may happen that a tof involving that quadcopter arrives, and without this check it will be saved to the tofStorage and live there "forever"
      locoId_t remoteNeighbourId = rxPacket->payload[i].id;
      if(findNeighbourData(neighboursStorage, remoteNeighbourId, false) != NULL) {
        tofData_t* tofData = findTofData(tofStorage, neighbourId, remoteNeighbourId, true);
        tofData->tof = (uint16_t)(rxPacket->payload[i].tof * clockCorrection);
      }
    }
  }

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.clockCorrectionCandidate = (uint32_t)(clockCorrectionCandidate * 1000000000);
  debug.clockCorrection = (uint32_t)(clockCorrection * 1000000000);

  debug.neighbourCount = countNeighbours(neighboursStorage);
#endif

  // Save the remoteTx, so we can use it to calculate the clockCorrection
  neighbourData->remoteTx = remoteTx;
  // Save the localRx, so we can calculate localReply when responding in the future
  neighbourData->localRx = localRx;

  updatePositionOf(neighbourData, isBuildingCoordinateSystem, neighboursStorage, tofStorage);
  updateOwnPosition(localId, neighbourId, neighbourData, tofStorage);

  // HACK for simulating 2D
  unsigned int neighbours = countNeighbours(neighboursStorage);
  if(neighbours <= 3) {
    // Set z coordinate to zero
    tofMeasurement_t tofData;
    tofData.timestamp = xTaskGetTickCount();
    tofData.distance = 0;
    tofData.stdDev = 0;
    estimatorKalmanEnqueueTOF(&tofData);
  }



#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  estimatorKalmanGetEstimatedPos(&debug.position);

  for(uint8_t i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    neighbourData_t* storage = &neighboursStorage[i];
    if(storage->id == neighbourData->id) {
      if(neighbourData->estimator.isInit) {
        point_t pos;
        estimatorKalmanEngine.getPosition(&neighbourData->estimator, &pos);
        debug.sendNeighbourPosition(pos, i);
      }
    }
  }

  if(neighbourData->estimator.isInit) {
    point_t pos;
    estimatorKalmanEngine.getPosition(&neighbourData->estimator, &pos);
    debug.localRx = pos.x;
  }
#endif
}

/**
 The standard deviation of a tof measurement. The value passed here (10) was obtained experimentally
 EDIT: the value is not anymore the standard deviation, but a random value obtained experimentally -- why?!
 */
#define OWN_STD_DEV 0.25
#define NEIGHBOURS_STD_DEV 0.25
#define STD_DEV_STABLE_THRESHOLD 0.1

void updatePositionOf(neighbourData_t* neighbourData, bool* isBuildingCoordinateSystem, neighbourData_t neighboursStorage[], tofData_t tofStorage[]) {
  // Give the initial state of the drone if needed
  if(!neighbourData->estimator.isInit) {
    /*
     In order to avoid trilaterating an initial position, we better pass a "fake" value with very high standard deviation and wait until further meassurements stabilize the position estimation
     */

    // In Meters
    float positionStdDev = 0.5;
    const vec3Measurement_t initialPosition = {
      // We give an initial value of (10, 10, 10) instead of (0, 0, 0) in order to make sure that, from the possible trilateration solutions, the system chooses the one in the positive axis. Also, the stdDev is not so high for the same pourpose
      .value = { .x = 10, .y = 10, .z = 10 },
      .stdDev = { .x = positionStdDev, .y = positionStdDev, .z = positionStdDev }
    };
    // In m/s
    float velocityStdDev = estimatorKalmanEngine.maximumAbsoluteVelocity;
    const vec3Measurement_t initialVelocity = {
      .value = { .x = 0, .y = 0, .z = 0 },
      .stdDev = { .x = velocityStdDev, .y = velocityStdDev, .z = velocityStdDev }
    };
    // In rad/s. We will not be using any attitude information, so set it as flat with minimum stdDev
    const vec3Measurement_t initialAttitudeError = {
      .value = { .x = 0, .y = 0, .z = 0 },
      .stdDev = { .x = 0, .y = 0, .z = 0 }
    };

    estimatorKalmanEngine.init(&neighbourData->estimator, &initialPosition, &initialVelocity, &initialAttitudeError);
  }

  unsigned int neighbours = countNeighbours(neighboursStorage);

  // Assume some position values while building the coordinate system
  if(*isBuildingCoordinateSystem) {
    if(neighbours == 1) {
      // Simulate 0D: the only known drone will be (0, 0, 0)
      positionMeasurement_t position = { .x = 0, .y = 0, .z = 0, .stdDev = 0 };
      estimatorKalmanEngine.enqueuePosition(&neighbourData->estimator, &position);
    } if(neighbours == 2) {
      // Simulate 1D
      positionMeasurement_t position = { .x = NAN, .y = 0, .z = 0, .stdDev = 0 };
      estimatorKalmanEngine.enqueuePosition(&neighbourData->estimator, &position);
    } else if (neighbours == 3) {
      // Simulate 2D
      positionMeasurement_t position = { .x = NAN, .y = NAN, .z = 0, .stdDev = 0 };
      estimatorKalmanEngine.enqueuePosition(&neighbourData->estimator, &position);
    } else if(neighbours > 3) {
      *isBuildingCoordinateSystem = false;
    }
  }

  distanceMeasurement_t distances[NEIGHBOUR_STORAGE_CAPACITY];
  uint8_t distancesIndex = 0;

  // Get all the positions and distances from the drone we want to update position, to the rest of them
  for (unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    if (neighboursStorage[i].isInitialized) {
      if(neighbourData->id != neighboursStorage[i].id) {
        tofData_t* tofData = findTofData(tofStorage, neighbourData->id, neighboursStorage[i].id, false);

        if (tofData != NULL) {
          estimatorKalmanStorage_t* estimator = &neighboursStorage[i].estimator;
          bool estimatorIsInit = estimator->isInit;
          bool positionIsStable = estimatorKalmanEngine.isPositionStable(estimator, STD_DEV_STABLE_THRESHOLD);

          if(estimatorIsInit && positionIsStable) {
            // Run update without new data, to get the most updated position (basically just run the prediction)
            // estimatorKalmanEngine.update(estimator);

            point_t positionData;
            estimatorKalmanEngine.getPosition(estimator, &positionData);

            distances[distancesIndex].x = positionData.x;
            distances[distancesIndex].y = positionData.y;
            distances[distancesIndex].z = positionData.z;
            distances[distancesIndex].distance = tofData->tof * TOF_TO_DISTANCE;
            distances[distancesIndex].stdDev = NEIGHBOURS_STD_DEV;
            distancesIndex++;
          }
        }
      }
    }
  }

  // Pass the known data to the estimator and enqueue it, so it can calculate the position in the next update
  for(unsigned int i = 0; i < distancesIndex; i++) {
    estimatorKalmanEngine.enqueueDistance(&neighbourData->estimator, &distances[i]);
  }

  // Use all the enqueued data to calculate the position
  estimatorKalmanEngine.update(&neighbourData->estimator, false);
}

void updateOwnPosition(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, tofData_t tofStorage[]) {
  bool estimatorIsInit = neighbourData->estimator.isInit;
  bool positionIsStable = estimatorKalmanEngine.isPositionStable(&neighbourData->estimator, STD_DEV_STABLE_THRESHOLD);

  if(estimatorIsInit && positionIsStable) {
    point_t remotePosition;
    estimatorKalmanEngine.getPosition(&neighbourData->estimator, &remotePosition);

    tofData_t* tofData = findTofData(tofStorage, localId, remoteId, false);
    if(tofData != NULL) {
      distanceMeasurement_t dist = {
        .distance = tofData->tof * TOF_TO_DISTANCE,
        .x = remotePosition.x,
        .y = remotePosition.y,
        .z = remotePosition.z,
        .stdDev = OWN_STD_DEV
      };
      estimatorKalmanEnqueueDistance(&dist);
    }
  }
}
