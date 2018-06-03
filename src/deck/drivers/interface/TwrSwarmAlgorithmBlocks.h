#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

locoId_t getId(locoAddress_t address);
uint32_t adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);
double calculateClockCorrection(uint64_t prevRemoteTx, uint64_t remoteTx, uint64_t prevLocalRx, uint64_t localRx);
neighbourData_t* getDataForNeighbour(dict* dct, locoId_t id);
unsigned int createTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoId_t sourceId, uint64_t localTx);
void processRxPacket(dwDevice_t *dev, locoId_t localId, lpsSwarmPacket_t* rxPacket, dict* dct, uint64_t lastKnownLocalTxTimestamp);
#endif /* TwrSwarmAlgorithmBlocks_h */
