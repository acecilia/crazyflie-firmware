#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

locoId_t generateId(void);
locoId_t generateIdNotInPacket(lpsSwarmPacket_t* packet);
void adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);
neighbourData_t* getDataForNeighbour(dict* dct, locoId_t id);
unsigned int allocAndFillTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoId_t sourceId);
void processRxPacket(dwDevice_t *dev, locoId_t localId, lpsSwarmPacket_t* rxPacket, dict* dct, uint64_t lastKnownLocalTxTimestamp);
#endif /* TwrSwarmAlgorithmBlocks_h */
