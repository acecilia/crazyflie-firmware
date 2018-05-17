#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

#include "libdw1000.h"
#include "locodeck.h"

typedef struct {
  locoAddress_t sourceAddress;
  uint64_t timeSinceLastSentPackage;
  uint64_t processingTime;
  // uint8_t data[30];
} lpsSwarmPacket_t;

typedef struct {
  void (*init)(void);
  void (*initiateRanging)(dwDevice_t *dev);
  uint32_t (*rxcallback)(dwDevice_t *dev, lpsSwarmPacket_t *packet, lpsAlgoOptions_t* options);
  void (*txcallback)(dwDevice_t *dev);
} twrSwarmAlgorithm_t;

extern twrSwarmAlgorithm_t twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
