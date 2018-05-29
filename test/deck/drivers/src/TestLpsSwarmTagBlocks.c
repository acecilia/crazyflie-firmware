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
  return hashtable2_dict_new(dict_uint64_cmp, dict_uint64_hash, 10);
}

/**
 Add an entry to the dictionary
 */
static void testFillDictionary(dict* dct, locoAddress_t address, neighbourData_t data) {
  neighbourData_t* result = getDataForNeighbour(dct, address);
  *result = data;
}

/**
 A function to find a pair inside the payload of a packet. Needed because when passing the dictionary data to an array, there order is not specified
 */
static addressTimePair_t* testFindPairInPayload(lpsSwarmPacket_t* packet, locoAddress_t address) {

  for (unsigned int i = 0; i < packet->rxLength; i++) {
    if (packet->rx[i].address == address) {
      return &packet->rx[i];
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
  uint32_t prevRemoteTx = 1000;
  uint32_t remoteTx = 2000;

  // Local clock running at slightly different frequency as remote
  uint32_t prevLocalRx = prevRemoteTx * expectedClockCorrection;
  uint32_t localRx = remoteTx * expectedClockCorrection;

  double result = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);

  TEST_ASSERT_EQUAL_UINT32(expectedClockCorrection, result);
}

/**
 Simulates that this is the first package coming, and there is no previous data: prevRemoteTx = 0 and prevLocalRx = 0
 */
void testCalculateClockCorrectionWithInvalidInputData() {
  double clockCorrection = 1.005;
  double expectedClockCorrection = 1; // Clock correction can not be calculated without prevRemoteTx and prevLocalRx, so the clock correction should report no frequency difference between local and remote clocks

  // Remote values
  uint32_t prevRemoteTx = 0;
  uint32_t remoteTx = 2000;

  // Local clock running at slightly different frequency as remote
  uint32_t prevLocalRx = 0;
  uint32_t localRx = remoteTx * clockCorrection;

  double result = calculateClockCorrection(prevRemoteTx, remoteTx, prevLocalRx, localRx);

  TEST_ASSERT_EQUAL_UINT32(expectedClockCorrection, result);
}

void testGetDataForNeighbourWithEmptyDictionary() {
  dict* dct = testCreateTestDictionary();

  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT32(0, result->localRx);
  TEST_ASSERT_EQUAL_UINT32(0, result->remoteTx);
  TEST_ASSERT_EQUAL_UINT32(0, result->tof);
}

void testGetDataForNeighbourWithFilledDictionary() {
  dict* dct = testCreateTestDictionary();
  neighbourData_t initialData = { .localRx = 1, .remoteTx = 2, .tof = 3 };

  testFillDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT32(initialData.localRx, result->localRx);
  TEST_ASSERT_EQUAL_UINT32(initialData.remoteTx, result->remoteTx);
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
  uint32_t localTx = 12345;
  locoAddress_t sourceAddress = 6;

  // Create package
  lpsSwarmPacket_t* txPacket = NULL;
  unsigned int txPacketLength = createTxPacket(&txPacket, dct, sourceAddress, localTx);

  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacket_t);

  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT64(sourceAddress, txPacket->sourceAddress);
  TEST_ASSERT_EQUAL_UINT32(localTx, txPacket->tx);
  TEST_ASSERT_EQUAL_UINT32(0, txPacket->rxLength);
}

void testCreateTxPacketWithPayload() {
  dict* dct = testCreateTestDictionary();
  uint32_t localTx = 12345;
  locoAddress_t sourceAddress = 6;

  // Fill dictionary
  unsigned int elementsCount = 5;
  for (unsigned int i = 0; i < elementsCount; i++) {
    neighbourData_t data = { .localRx = i, .remoteTx = i + 1, .tof = i + 2 };
    testFillDictionary(dct, i, data);
  }
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacket_t) + elementsCount * sizeof(addressTimePair_t);

  // Create package
  lpsSwarmPacket_t* txPacket = NULL;
  unsigned int txPacketLength = createTxPacket(&txPacket, dct, sourceAddress, localTx);

  // Verify result
  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT64(sourceAddress, txPacket->sourceAddress);
  TEST_ASSERT_EQUAL_UINT32(localTx, txPacket->tx);
  TEST_ASSERT_EQUAL_UINT32(elementsCount, txPacket->rxLength);
  for (unsigned int i = 0; i < elementsCount; i++) {
    addressTimePair_t* pair = testFindPairInPayload(txPacket, i);
    TEST_ASSERT_EQUAL_UINT64(i, pair->address);
    TEST_ASSERT_EQUAL_UINT32(i, pair->time);
  }
}

void testProcessRxPacket() {
  // Event timestamps:
  // * prevRemoteTx                                 ---> prevlocalRx = (clockCorrection * prevRemoteTx) + tof
  // * remoteRx = (localTx + tof) / clockCorrection <--- localTx = prevlocalRx + localReply
  // * remoteTx = remoteRx + remoteReply            ---> localRx = (2 * remoteTx) + tof

  // Expected values: giving them pair values we ensure proper integer operations (mainly division) when calculating the actual values
  uint32_t tof = 4;             // Tof meassured by local clock
  uint32_t clockCorrection = 2; // Local clock works <clockCorrection> times faster than remote clock
  uint32_t localReply = 4;      // Local reply time meassured by local clock
  uint32_t remoteReply = 2;     // Remote reply time meassured by Remote clock

  // Actual values
  uint32_t prevRemoteTx = 10;
  uint32_t prevlocalRx = (clockCorrection * prevRemoteTx) + tof;
  uint32_t localTx = prevlocalRx + localReply;
  uint32_t remoteRx = (localTx + tof) / clockCorrection;
  uint32_t remoteTx = remoteRx + remoteReply;
  uint32_t localRx = (2 * remoteTx) + tof;

  //Addresses
  locoAddress_t sourceAddress = 5;
  locoAddress_t ownAddress = 6;

  // Create the context
  ctx_s ctx = {
    .dct = testCreateTestDictionary(),
    .localTx = localTx
  };

  // Set previous data
  neighbourData_t* prevNeighbourData = getDataForNeighbour(ctx.dct, sourceAddress);
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
  lpsSwarmPacket_t* rxPacket = pvPortMalloc(sizeof(lpsSwarmPacket_t) + 1 * sizeof(addressTimePair_t));
  rxPacket->sourceAddress = sourceAddress;
  rxPacket->tx = remoteTx;
  rxPacket->rxLength = 1;
  addressTimePair_t pair = {
    .address = ownAddress,
    .time = remoteRx
  };
  rxPacket->rx[0] = pair;

  // Test
  processRxPacket(&dummyDev, ownAddress, rxPacket, &ctx);

  // Verify result
  neighbourData_t* currentNeighbourData = getDataForNeighbour(ctx.dct, sourceAddress);
  TEST_ASSERT_EQUAL_UINT32(remoteTx, currentNeighbourData->remoteTx);
  TEST_ASSERT_EQUAL_UINT32(localRx, currentNeighbourData->localRx);
  TEST_ASSERT_EQUAL_UINT32(tof, currentNeighbourData->tof);
}

