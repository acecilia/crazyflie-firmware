#include "TwrSwarmAlgorithm.h"
#include "TwrSwarmAlgorithmBlocks.h"
#include "randomizedTimerEngine.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

#include "debug.h"

#define PACKET_TYPE_SWARM 0x47

// TODO: remove and do it smarter
#define ANTENNA_OFFSET_METERS 154.6
#define ANTENNA_DELAY (uint16_t)((ANTENNA_OFFSET_METERS * LOCODECK_TS_FREQ) / SPEED_OF_LIGHT)
#define ANTENNA_ACCEPTED_NOISE_ERROR 100
uint16_t antennaDelay = ANTENNA_DELAY;

static lpsSwarmPacket_t packet;

/**
 A type that encapsulates all the required global values of the algorithm
 */
static struct {
  neighbourData_t neighboursStorage[NEIGHBOUR_STORAGE_CAPACITY];
  tofData_t tofStorage[TOF_STORAGE_CAPACITY];

  locoId_t localId;
  uint8_t nextTxSeqNr; // Local sequence number of the transmitted packets

  uint32_t timeOfNextTx;
  uint32_t averageTxDelay;

  bool isBuildingCoordinateSystem;

  xTimerHandle timer;
} ctx;

/* Helpers */
/**********************************/

static void setRxMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t now() {
  return xTaskGetTickCount();
}

/**********************************/

/**
 Callback for the timer. Used to update variables of the algorithm periodically
 */
static void timerCallback(xTimerHandle timer) {
  // Adjust average tx delay based on the number of known drones around
  ctx.averageTxDelay = calculateAverageTxDelay(ctx.neighboursStorage);

  removeOutdatedData(ctx.neighboursStorage, ctx.tofStorage);

  DEBUG_PRINT("N: %d\n", countNeighbours(ctx.neighboursStorage));
  DEBUG_PRINT("Tof: %d\n", countTof(ctx.tofStorage));
}

/**
 Initialize all the required variables of the algorithm
 */
static void init(dwDevice_t *dev) {
  initRandomizationEngine(dev);

  // Initialize neighbours storage
  for(unsigned int i = 0; i < NEIGHBOUR_STORAGE_CAPACITY; i++) {
    ctx.neighboursStorage[i].isInitialized = false;
  }

  // Initialize tof storage
  for(unsigned int i = 0; i < TOF_STORAGE_CAPACITY; i++) {
    ctx.tofStorage[i].isInitialized = false;
  }

  ctx.localId = generateId();
  DEBUG_PRINT("Swarm Id: %d\n", ctx.localId);

  ctx.nextTxSeqNr = 0;

  // Related with random transmission
  ctx.averageTxDelay = calculateAverageTxDelay(0);
  ctx.timeOfNextTx = calculateRandomDelayToNextTx(ctx.averageTxDelay);

  // Starts without a coordinate system: needs to build it
  ctx.isBuildingCoordinateSystem = true;

  // Timer to execute actions periodically
  ctx.timer = xTimerCreate("timer", M2T(1000), pdTRUE, NULL, timerCallback);
  xTimerStart(ctx.timer, 0);
}

/**
 Fill and transmit a packet
 */
static void transmit(dwDevice_t *dev) {
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.totalRangingPerSec++;
#endif

  lpsSwarmPacket_t* txPacket = &packet;
  txPacket->header.type = PACKET_TYPE_SWARM;

  setTxData(txPacket, ctx.localId, &ctx.nextTxSeqNr, ctx.neighboursStorage, ctx.tofStorage);
  unsigned int packetSize = calculatePacketSize(txPacket);

  // Set tx time inside txPacket
  dwTime_t tx = findTransmitTimeAsSoonAsPossible(dev);
  uint64_t localTx = tx.full;
  txPacket->header.tx = localTx;

  // Set data
  dwSetData(dev, (uint8_t*)txPacket, packetSize);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, tx);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

/**
 Handle the data coming in a received packet
 */
static void handleRxPacket(dwDevice_t *dev) {
  // If the received packet is not of our type, discard it
  if(packet.header.type != PACKET_TYPE_SWARM) {
    return;
  }


  // Makes sure the id is unique among the neighbours around, and regenerate it only if the local ranging information is less than the information coming on the packet
  if (packet.header.sourceId == ctx.localId) {
    ctx.localId = generateIdNotInPacket(ctx.neighboursStorage, &packet);
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
    debug.idFailure++;
#endif
  }

  processRxPacket(dev, ctx.localId, &packet, antennaDelay, &ctx.isBuildingCoordinateSystem, ctx.neighboursStorage, ctx.tofStorage);
}

/**
 Called for each DW radio event
 */
static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  unsigned int dataLength = 0;

  // If data was received, get it before changing the status of the DW1000 chip
  if (event == eventPacketReceived) {
    dataLength = dwGetDataLength(dev);
    dwGetData(dev, (uint8_t*)&packet, dataLength);
  }

  // Configure the DW1000 for Rx before processing the event: we want the chip on Rx mode as much time as possible, to avoid losing packets
  setRxMode(dev);

  // Process the event
  if (event == eventPacketReceived && dataLength > 0) {
    handleRxPacket(dev);
  } else if (event == eventTimeout) {
    ctx.timeOfNextTx = now() + calculateRandomDelayToNextTx(ctx.averageTxDelay);
    transmit(dev);
  }

  int32_t timeoutForNextTx = (int32_t)(ctx.timeOfNextTx - now());
  if (timeoutForNextTx < 0) {
    // Force a timeout, that will result in a new call to onEvent
    timeoutForNextTx = 0;
  }
  return (uint32_t)timeoutForNextTx;
}

twrSwarmAlgorithm_t twrSwarmAlgorithm = {
  .init = init,
  .onEvent = onEvent
};
