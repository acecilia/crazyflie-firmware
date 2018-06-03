// All the files that we want to test should be included here: Ceedling will scan this header file (.h), and compile all the needed source files (.c). Ceedling will look for this source files (.c) in the included paths (in the [includes:][items:] entry of the yml configuration file passed to cmock). Not including ALL the files to be tested here as <<#include "fileToBeTested.h">> will result in "Undefined symbols" errors when linking. From the Ceedling docs: "Ceedling knows what files to compile and link into each individual test executable by way of the #include list contained in each test file. Any C source files in the configured search directories that correspond to the header files included in a test file will be compiled and linked into the resulting test fixture executable. From this same #include list, Ceedling knows which files to mock and compile and link into the test executable (if you use mocks in your tests). That was a lot of clauses and information in a very few sentences; the example that follows in a bit will make it clearer." See: https://github.com/ThrowTheSwitch/Ceedling/blob/master/docs/CeedlingPacket.md

#include "TwrSwarmAlgorithmBlocks.h"
#include "libdict.h"
#include "dict.h"
#include "hashtable.h"
#include "hashtable2.h"
#include "hb_tree.h"
#include "pr_tree.h"
#include "rb_tree.h"
#include "skiplist.h"
#include "sp_tree.h"
#include "tr_tree.h"
#include "wb_tree.h"
#include "hashtable_common.h"
#include "tree_common.h"

#include "unity.h"

#include "mock_libdw1000.h"

#include "freertosMocks.h"

// Test helper functions

/**
 Create a dictionary. Used for avoiding code duplication
 */
static dict* testCreateTestDictionary() {
  return hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10);
}

/**
 Add an entry to the dictionary
 */
static void testFillDictionary(dict* dct, locoId_t id, neighbourData_t data) {
  neighbourData_t* result = getDataForNeighbour(dct, id);
  *result = data;
}

/**
 A function to find a pair inside the payload of a packet. Needed because when passing the dictionary data to an array, there order is not specified
 */
static payload_t* testFindPairInPayload(lpsSwarmPacket_t* packet, locoId_t id) {
  for (unsigned int i = 0; i < packet->payloadLength; i++) {
    if (packet->payload[i].id == id) {
      return &packet->payload[i];
    }
  }
  return NULL;
}

// Tests

void setUp(void) {
}

void tearDown(void) {
}

void testAdjustTxRxTime() {
  uint32_t initialValue = 0x000001FF;
  uint32_t expectedValue = 0x00000200;
  uint32_t expectedAdded = expectedValue - initialValue;

  dwTime_t time = { .full = initialValue };
  uint32_t added = adjustTxRxTime(&time);

  TEST_ASSERT_EQUAL_UINT32(expectedValue, time.low32);
  TEST_ASSERT_EQUAL_UINT32(expectedAdded, added);
}

void testFindTransmitTimeAsSoonAsPossible() {
  dwTime_t initialValue = { .full = 12345 };
  dwTime_t expectedValue = { .full = 27568640 };

  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetSystemTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dummyDev);
  
  TEST_ASSERT_EQUAL_UINT64(expectedValue.full, time.full);
}

void testCalculateClockCorrectionWithValidInputData() {
  double expectedClockCorrection = 1.005;

  // Remote values
  uint64_t prevRemoteTx = 1000;
  uint64_t remoteTx = 2000;

  // Local clock running at slightly different frequency as remote
  uint64_t prevLocalRx = prevRemoteTx * expectedClockCorrection;
  uint64_t localRx = remoteTx * expectedClockCorrection;

  double result = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);

  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
  TEST_ASSERT_NOT_EQUAL(prevRemoteTx, prevLocalRx);
  TEST_ASSERT_NOT_EQUAL(remoteTx, localRx);
}

/**
 Simulates that this is the first package coming, and there is no previous data: prevRemoteTx = 0 and prevLocalRx = 0
 */
void testCalculateClockCorrectionWithInvalidInputData() {
  double clockCorrection = 1.005;
  double expectedClockCorrection = 1; // Clock correction can not be calculated without prevRemoteTx and prevLocalRx, so the clock correction should report no frequency difference between local and remote clocks

  // Remote values
  uint64_t prevRemoteTx = 0;
  uint64_t remoteTx = 2000;

  // Local clock running at slightly different frequency as remote
  uint64_t prevLocalRx = 0;
  uint64_t localRx = remoteTx * clockCorrection;

  double result = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);

  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
  TEST_ASSERT_NOT_EQUAL(remoteTx, localRx);
}

void testGetDataForNeighbourWithEmptyDictionary() {
  dict* dct = testCreateTestDictionary();

  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT64(0, result->localRx);
  TEST_ASSERT_EQUAL_UINT64(0, result->remoteTx);
  TEST_ASSERT_EQUAL_UINT64(0, result->tof);
}

void testGetDataForNeighbourWithFilledDictionary() {
  dict* dct = testCreateTestDictionary();
  neighbourData_t initialData = { .localRx = 1, .remoteTx = 2, .tof = 3 };

  testFillDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT64(initialData.localRx, result->localRx);
  TEST_ASSERT_EQUAL_UINT64(initialData.remoteTx, result->remoteTx);
  TEST_ASSERT_EQUAL_UINT32(initialData.tof, result->tof);
  TEST_ASSERT_NOT_EQUAL(&initialData, result);
}

void testGetDataForNeighbourWithFilledDictionaryAndDifferentKeys() {
  dict* dct = testCreateTestDictionary();
  neighbourData_t initialData = { .localRx = 1, .remoteTx = 2, .tof = 3 };

  testFillDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 2);

  TEST_ASSERT_NOT_EQUAL(initialData.localRx, result->localRx);
  TEST_ASSERT_NOT_EQUAL(initialData.remoteTx, result->remoteTx);
  TEST_ASSERT_NOT_EQUAL(initialData.tof, result->tof);
  TEST_ASSERT_NOT_EQUAL(&initialData, result);
}

void testCreateTxPacketWithoutPayload() {
  dict* dct = testCreateTestDictionary();
  uint64_t localTx = 12345;
  locoId_t sourceId = 6;
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacket_t);

  // Create package
  lpsSwarmPacket_t* txPacket = NULL;
  unsigned int txPacketLength = createTxPacket(&txPacket, dct, sourceId, localTx);

  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(sourceId, txPacket->sourceId);
  TEST_ASSERT_EQUAL_UINT64(localTx, txPacket->tx);
  TEST_ASSERT_EQUAL_UINT8(0, txPacket->payloadLength);
}

void testCreateTxPacketWithPayload() {
  dict* dct = testCreateTestDictionary();
  uint64_t localTx = 12345;
  locoId_t sourceId = 6;

  // Fill dictionary
  uint8_t elementsCount = 5;
  for (uint8_t i = 0; i < elementsCount; i++) {
    neighbourData_t data = { .localRx = i, .remoteTx = i + 1, .tof = i + 2 };
    testFillDictionary(dct, i, data);
  }
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacket_t) + elementsCount * sizeof(payload_t);

  // Create package
  lpsSwarmPacket_t* txPacket = NULL;
  unsigned int txPacketLength = createTxPacket(&txPacket, dct, sourceId, localTx);

  // Verify result
  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(sourceId, txPacket->sourceId);
  TEST_ASSERT_EQUAL_UINT64(localTx, txPacket->tx);
  TEST_ASSERT_EQUAL_UINT8(elementsCount, txPacket->payloadLength);
  for (uint8_t i = 0; i < elementsCount; i++) {
    payload_t* pair = testFindPairInPayload(txPacket, i);
    TEST_ASSERT_EQUAL_UINT8(i, pair->id);
    TEST_ASSERT_EQUAL_UINT64(i, pair->time);
  }
}

void testProcessRxPacketWithPayload() {
  // Event timestamps:
  // * prevRemoteTx                                 ---> prevlocalRx = (clockCorrection * prevRemoteTx) + tof
  // * remoteRx = (localTx + tof) / clockCorrection <--- localTx = prevlocalRx + localReply
  // * remoteTx = remoteRx + remoteReply            ---> localRx = (clockCorrection * remoteTx) + tof

  // Expected values: giving them pair values we ensure proper integer operations (mainly division) when calculating the actual values
  uint32_t tof = 4 * 10000;                             // Tof meassured by local clock
  double clockCorrection = 1.15;                        // Local clock works <clockCorrection> times faster than remote clock
  uint32_t localReply = 4 * 10000;                      // Local reply time meassured by local clock
  uint32_t remoteReply = localReply / clockCorrection;  // Remote reply time meassured by Remote clock

  // Timestamp values
  uint64_t prevRemoteTx = 20500;
  uint64_t prevlocalRx = (clockCorrection * prevRemoteTx) + tof;
  uint64_t localTx = prevlocalRx + localReply;
  uint64_t remoteRx = (localTx + tof) / clockCorrection;
  uint64_t remoteTx = remoteRx + remoteReply;
  uint64_t localRx = (clockCorrection * remoteTx) + tof;

  //Ids
  locoId_t remoteId = 5;
  locoId_t localId = 6;

  // Create the dictionary
  dict* dct = testCreateTestDictionary();

  // Set previous data
  neighbourData_t* prevNeighbourData = getDataForNeighbour(dct, remoteId);
  prevNeighbourData->remoteTx = prevRemoteTx;
  prevNeighbourData->localRx = prevlocalRx;

  // Setup the mock in charge of generating localRx
  dwTime_t localRxTimeStamp = { .full = localRx };
  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetReceiveTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetReceiveTimestamp_IgnoreArg_dev();
  dwGetReceiveTimestamp_IgnoreArg_time();
  dwGetReceiveTimestamp_ReturnThruPtr_time(&localRxTimeStamp);

  // Create and fill rxPacket
  lpsSwarmPacket_t* rxPacket = pvPortMalloc(sizeof(lpsSwarmPacket_t) + 1 * sizeof(payload_t));
  rxPacket->sourceId = remoteId;
  rxPacket->tx = remoteTx;
  rxPacket->payloadLength = 1;
  payload_t pair = {
    .id = localId,
    .time = remoteRx
  };
  rxPacket->payload[0] = pair;

  // Test
  processRxPacket(&dummyDev, localId, rxPacket, dct, localTx);

  // Verify result
  neighbourData_t* currentNeighbourData = getDataForNeighbour(dct, remoteId);
  TEST_ASSERT_EQUAL_UINT64(remoteTx, currentNeighbourData->remoteTx);
  TEST_ASSERT_EQUAL_UINT64(localRx, currentNeighbourData->localRx);
  TEST_ASSERT_EQUAL_UINT32(tof - 1, currentNeighbourData->tof); // The -1 is required, because in the conversion between double and integer the system cuts the number down
}

void testProcessRxPacketWithoutPayload() {
  // Timestamp values
  uint64_t localRx = 12345;
  uint64_t remoteTx = 23456;

  //Ids
  locoId_t remoteId = 5;
  locoId_t localId = 6;

  // Create the dictionary
  dict* dct = testCreateTestDictionary();

  // Setup the mock in charge of generating localRx
  dwTime_t localRxTimeStamp = { .full = localRx };
  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetReceiveTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetReceiveTimestamp_IgnoreArg_dev();
  dwGetReceiveTimestamp_IgnoreArg_time();
  dwGetReceiveTimestamp_ReturnThruPtr_time(&localRxTimeStamp);

  // Create and fill rxPacket
  lpsSwarmPacket_t* rxPacket = pvPortMalloc(sizeof(lpsSwarmPacket_t));
  rxPacket->sourceId = remoteId;
  rxPacket->tx = remoteTx;
  rxPacket->payloadLength = 0;

  // Test
  uint64_t dummyLocalTx = 9999; // Not needed, as localTx is only used when there is payload
  processRxPacket(&dummyDev, localId, rxPacket, dct, dummyLocalTx);

  // Verify result
  neighbourData_t* currentNeighbourData = getDataForNeighbour(dct, remoteId);
  TEST_ASSERT_EQUAL_UINT64(remoteTx, currentNeighbourData->remoteTx);
  TEST_ASSERT_EQUAL_UINT64(localRx, currentNeighbourData->localRx);
}
