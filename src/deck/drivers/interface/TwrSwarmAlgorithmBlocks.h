#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

uint32_t adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);
double calculateClockCorrection(uint64_t prevRemoteTx, uint64_t remoteTx, uint64_t prevLocalRx, uint64_t localRx);
neighbourData_t* getDataForNeighbour(dict* dct, locoAddress_t address);
unsigned int createTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoAddress_t sourceAddress, uint64_t localTx);
void processRxPacket(dwDevice_t *dev, locoAddress_t ownAddress, lpsSwarmPacket_t* rxPacket, dict* dct, uint64_t localTx);
#endif /* TwrSwarmAlgorithmBlocks_h */
