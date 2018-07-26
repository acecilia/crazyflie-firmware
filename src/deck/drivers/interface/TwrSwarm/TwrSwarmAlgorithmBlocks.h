#ifndef TwrSwarmAlgorithmBlocks_h
#define TwrSwarmAlgorithmBlocks_h

#include "TwrSwarmAlgorithm.h"

uint32_t calculateRandomDelayToNextTx(uint32_t averageTxDelay);
uint32_t calculateAverageTxDelay(neighbourData_t neighboursStorage[]);

bool verifySeqNr(const uint8_t seqNr, const uint8_t expectedSeqNr);

void initRandomizationEngine(dwDevice_t *dev);
locoId_t generateId(void);
locoId_t generateIdNotInPacket(neighbourData_t neighboursStorage[], lpsSwarmPacket_t* packet);
locoIdx2_t getHashFromIds(const locoId_t id1, const locoId_t id2);

void adjustTxRxTime(dwTime_t *time);
dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev);

tofData_t* findTofData(tofData_t storage[], const locoId_t id1, const locoId_t id2, const bool insertIfNotFound);
unsigned int countTof(tofData_t storage[]);
neighbourData_t* findNeighbourData(neighbourData_t storage[], const locoId_t id, const bool insertIfNotFound);
unsigned int countNeighbours(neighbourData_t storage[]);

unsigned int calculatePacketSize(lpsSwarmPacket_t* packet);
void setTxData(lpsSwarmPacket_t* txPacket, locoId_t sourceId, uint8_t* nextTxSeqNr, neighbourData_t neighboursStorage[], tofData_t tofStorage[]);
void processRxPacket(dwDevice_t *dev, locoId_t localId, const lpsSwarmPacket_t* rxPacket, const uint16_t antennaDelay, bool* isBuildingCoordinateSystem, neighbourData_t neighboursStorage[], tofData_t tofStorage[]);

void updatePositionOf(neighbourData_t* neighbourData, bool* isBuildingCoordinateSystem, neighbourData_t neighboursStorage[], tofData_t tofStorage[]);
void updateOwnPosition(locoId_t localId, locoId_t remoteId, neighbourData_t* neighbourData, tofData_t tofStorage[]);
#endif /* TwrSwarmAlgorithmBlocks_h */
