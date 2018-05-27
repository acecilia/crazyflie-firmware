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
  uint32_t time;
} addressTimePair_t;

/**
 A type that encapsulates the data included in a packet
 */
typedef struct {
  locoAddress_t sourceAddress;
  uint32_t tx;

  uint32_t rxLength;
  addressTimePair_t rx[];
} lpsSwarmPacket_t;

/**
 A type that encapsulates the information stored in the dictionary
 */
typedef struct {
  // Values to calculate clockDrift
  uint32_t localRx; // To be set after reception
  uint32_t remoteTx; // To be set after reception

  uint32_t tof; // To be set after reception
} neighbourData_t;

/**
 A type that encapsulates all the required global values of the algorithm
 */
typedef struct {
  dict *dct;

  // Values to calculate t_round
  uint32_t localTx; // To be set after transmission
} ctx_s;

typedef struct {
  void (*init)(void);
  void (*initiateRanging)(dwDevice_t *dev);
  uint32_t (*rxcallback)(dwDevice_t *dev, lpsAlgoOptions_t* options, lpsSwarmPacket_t* rxPacket, unsigned int dataLength);
  void (*txcallback)(dwDevice_t *dev);

  // Exposed for testing
  neighbourData_t* (*getDataForNeighbour)(dict* dct, locoAddress_t address);
  ctx_s* ctx;
} twrSwarmAlgorithm_t;

extern twrSwarmAlgorithm_t twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
