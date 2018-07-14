#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

uint32_t calculateRandomDelayToNextTx(uint32_t averageTxDelay);
uint32_t calculateAverageTxDelay(uint8_t numberOfNeighbours);

bool verifySeqNr(const uint8_t seqNr, const uint8_t expectedSeqNr);
locoIdx2_t getHashFromIds(const locoId_t id1, const locoId_t id2);
void initRandomizationEngine(dwDevice_t *dev);
locoId_t generateId(void);
locoId_t generateIdNotIn(lpsSwarmPacket_t* packet, dict* dct);
void adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);
tofData_t* findTof(tofData_t tofStorage[], const locoId_t id1, const locoId_t id2, const bool insertIfNotFound);
neighbourData_t* getDataForNeighbour(dict* dct, const locoId_t id, const bool insertIfNotFound);
unsigned int calculatePacketSize(lpsSwarmPacket_t* packet);
void setTxData(lpsSwarmPacket_t* txPacket, locoId_t sourceId, uint8_t* nextTxSeqNr, dict* neighbourDct, tofData_t tofStorage[]);
void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, dict* neighboursDct, tofData_t tofStorage[]);
void updatePositionOf(locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, tofData_t tofStorage[]);
void updateOwnPosition(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, dict* neighboursDct, tofData_t tofStorage[]);
#endif /* TwrSwarmAlgorithmBlocks_h */
