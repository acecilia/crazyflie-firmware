#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

#include "libdw1000.h"
#include "locodeck.h"

typedef struct {
  locoAddress_t address;
  uint32_t time;
} addressTimePair_t;

typedef struct {
  locoAddress_t sourceAddress;
  uint32_t tx;

  int rxLength;
  addressTimePair_t rx[];
} lpsSwarmPacket_t;

typedef struct {
  void (*init)(void);
  void (*initiateRanging)(dwDevice_t *dev);
  uint32_t (*rxcallback)(dwDevice_t *dev, lpsAlgoOptions_t* options);
  void (*txcallback)(dwDevice_t *dev);
} twrSwarmAlgorithm_t;

extern twrSwarmAlgorithm_t twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
