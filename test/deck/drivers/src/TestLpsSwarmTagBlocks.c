// All the files that we want to test should be included here: Ceedling will scan this header file (.h), and compile all the needed source files (.c). Ceedling will look for this source files (.c) in the included paths (in the [includes:][items:] entry of the yml configuration file passed to cmock). Not including ALL the files to be tested here as <<#include "fileToBeTested.h">> will result in "Undefined symbols" errors when linking. From the Ceedling docs: "Ceedling knows what files to compile and link into each individual test executable by way of the #include list contained in each test file. Any C source files in the configured search directories that correspond to the header files included in a test file will be compiled and linked into the resulting test fixture executable. From this same #include list, Ceedling knows which files to mock and compile and link into the test executable (if you use mocks in your tests). That was a lot of clauses and information in a very few sentences; the example that follows in a bit will make it clearer." See: https://github.com/ThrowTheSwitch/Ceedling/blob/master/docs/CeedlingPacket.md

#include "TwrSwarmAlgorithmBlocks.h"
#include "clockCorrectionEngine.h"
#include "clockCorrectionFunctions.h"
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

#define DW1000_MAXIMUM_COUNT (uint64_t)( 1099511627775 ) //The maximum timestamp the DW1000 can return (40 bits: 2^40 - 1)

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

void testAdjustTxRxTimeSmallNumber() {
  uint64_t initialValue = 0x000001FF;
  uint64_t expectedValue = 0x00000200;

  dwTime_t time = { .full = initialValue };
  adjustTxRxTime(&time);

  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
}

void testAdjustTxRxTimeBigNumber() {
  uint64_t initialValue = 0x0123456789ABCDEF;
  uint64_t expectedValue = 0x0123456789ABCE00;

  dwTime_t time = { .full = initialValue };
  adjustTxRxTime(&time);

  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
}

void testFindTransmitTimeAsSoonAsPossibleWithZeroInitialValue() {
  dwTime_t initialValue = { .full = 0 };
  uint64_t expectedValue = 27556352;

  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetSystemTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dummyDev);
  
  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
}

void testFindTransmitTimeAsSoonAsPossibleWithLowInitialValue() {
  uint64_t expectedValueWithZeroInitialValue = 27556352;
  dwTime_t initialValue = { .full = 40000000 };
  uint64_t expectedValue = expectedValueWithZeroInitialValue + initialValue.full;

  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetSystemTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dummyDev);

  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
}

void testFindTransmitTimeAsSoonAsPossibleWithHighInitialValueWithoutWrappingArround() {
  uint64_t expectedValueWithZeroInitialValue = 27556352;
  dwTime_t initialValue = { .full = DW1000_MAXIMUM_COUNT - expectedValueWithZeroInitialValue * 2 };
  uint64_t expectedValue = 1099484071424; // Approximatelly: expectedValueWithZeroInitialValue + initialValue.full = 1099484071423

  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetSystemTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dummyDev);

  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
}

void testFindTransmitTimeAsSoonAsPossibleHighInitialValueAndWrappingAround() {
  uint64_t expectedValueWithZeroInitialValue = 27556352;
  dwTime_t initialValue = { .full = DW1000_MAXIMUM_COUNT - expectedValueWithZeroInitialValue / 2 };
  uint64_t expectedValue = 13778944; // Approximatelly: expectedValueWithZeroInitialValue + initialValue.full (counting the wrapping) = expectedValueWithZeroInitialValue / 2 = 13778176

  dwDevice_t dummyDev;
  dwTime_t dummyValue;
  dwGetSystemTimestamp_Expect(&dummyDev, &dummyValue);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dummyDev);

  TEST_ASSERT_EQUAL_UINT64(expectedValue, time.full);
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

void testAllocAndFillTxPacketWithoutPayload() {
  dict* dct = testCreateTestDictionary();
  locoId_t sourceId = 6;
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacket_t);

  // Create package
  lpsSwarmPacket_t* txPacket = NULL;
  unsigned int txPacketLength = allocAndFillTxPacket(&txPacket, dct, sourceId);

  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(sourceId, txPacket->sourceId);
  TEST_ASSERT_EQUAL_UINT64(0, txPacket->tx);
  TEST_ASSERT_EQUAL_UINT8(0, txPacket->payloadLength);
}

void testAllocAndFillTxPacketWithPayload() {
  dict* dct = testCreateTestDictionary();
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
  unsigned int txPacketLength = allocAndFillTxPacket(&txPacket, dct, sourceId);

  // Verify result
  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(sourceId, txPacket->sourceId);
  TEST_ASSERT_EQUAL_UINT64(0, txPacket->tx);
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
  double clockCorrection = 1.00001;                     // Local clock works <clockCorrection> times faster than remote clock
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
  prevNeighbourData->clockCorrectionStorage.clockCorrection = 5; // A wrong value on pourpose, so the proper clock correction is set and then we can assert it
  prevNeighbourData->clockCorrectionStorage.clockCorrectionBucket = 0;

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
  TEST_ASSERT_DOUBLE_WITHIN(0.00001, clockCorrection, currentNeighbourData->clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(1, currentNeighbourData->clockCorrectionStorage.clockCorrectionBucket);
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
