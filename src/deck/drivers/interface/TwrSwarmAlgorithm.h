#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

#include "libdw1000.h"
#include "locodeck.h"
#include "libdict.h"

/**
 A type that relates an address with some associated information
 */
typedef struct {
  locoAddress_t address;
  uint64_t time;
} __attribute__((packed)) addressTimePair_t;

/**
 A type that encapsulates the data included in a packet
 */
typedef struct {
  locoAddress_t sourceAddress;
  uint64_t tx;

  uint8_t payloadLength; // Allows a logic limit of 256 pairs (which limits the maximum number of drones participating in the swarm to 256)
  addressTimePair_t payload[];
} __attribute__((packed)) lpsSwarmPacket_t;

/**
 A type that encapsulates the information stored in the dictionary
 */
typedef struct {
  // Values to calculate clockDrift
  uint64_t localRx; // To be set after reception
  uint64_t remoteTx; // To be set after reception

  uint32_t tof; // To be set after reception
} __attribute__((packed)) neighbourData_t;

typedef struct {
  void (*init)(void);
  void (*initiateRanging)(dwDevice_t *dev);
  uint32_t (*rxcallback)(dwDevice_t *dev, lpsAlgoOptions_t* options, lpsSwarmPacket_t* rxPacket, unsigned int dataLength);
  void (*txcallback)(dwDevice_t *dev);
} twrSwarmAlgorithm_t;

extern twrSwarmAlgorithm_t twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
