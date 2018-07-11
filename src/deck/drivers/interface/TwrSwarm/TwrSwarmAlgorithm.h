#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

#include "libdw1000.h"
#include "locodeck.h"
#include "libdict.h"
#include "clockCorrectionStorage.h"
#include "estimatorKalmanStorage.h"

typedef uint8_t locoId_t;
typedef uint16_t locoIdx2_t;

/**
 A type that relates the id of a drone with some associated information
 */
typedef struct {
  locoId_t id;
  uint64_t rx;
  uint64_t tx;
  uint16_t tof;
} __attribute__((packed)) payload_t;

/**
 A type that encapsulates the data included in the header of a packet
 */
typedef struct {
  locoId_t sourceId;
  uint8_t seqNr; // Sequence number of this packet
  uint64_t tx; // TODO: see if we can reduce the size of data type

  uint8_t payloadLength; // Allows a logic limit of 256 pairs (which limits the maximum number of drones participating in the swarm to 256)
} __attribute__((packed)) lpsSwarmPacketHeader_t;

/**
 A type that encapsulates the data included in a packet
 */
typedef struct {
  lpsSwarmPacketHeader_t header;

  uint8_t payload[128]; // TODO: see what is really the limit
} __attribute__((packed)) lpsSwarmPacket_t;

/**
 A type that encapsulates the information to keep about the neighbours
 */
typedef struct {
  // Values to calculate clockCorrection
  uint64_t localRx; // To be set after reception
  uint64_t remoteTx; // To be set after reception

  clockCorrectionStorage_t clockCorrectionStorage;
  uint8_t expectedSeqNr; // Expected sequence number for the received packet

  estimatorKalmanStorage_t estimatorKalmanStorage; // Used to obtain the coordinates of the neighbour
} __attribute__((packed)) neighbourData_t;

/**
 A type that encapsulates the information to keep track of the tof
 */
typedef struct {
  uint16_t tof; // To be set after reception
} __attribute__((packed)) tofData_t;

typedef struct {
  void (*init)(dwDevice_t *dev);
  uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
} twrSwarmAlgorithm_t;

extern twrSwarmAlgorithm_t twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
