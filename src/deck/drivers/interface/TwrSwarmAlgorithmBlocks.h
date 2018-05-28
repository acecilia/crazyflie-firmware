#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

uint32_t adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);
double calculateClockCorrection(uint32_t prevRemoteTx, uint32_t remoteTx, uint32_t prevLocalRx, uint32_t localRx);
neighbourData_t* getDataForNeighbour(dict* dct, locoAddress_t address);
unsigned int createTxPacket(lpsSwarmPacket_t** txPacketPointer, dict* dct, locoAddress_t sourceAddress, uint32_t localTx);
void processRxPacket(dwDevice_t *dev, locoAddress_t ownAddress, lpsSwarmPacket_t* rxPacket, ctx_s* ctx);

#endif /* TwrSwarmAlgorithmBlocks_h */
