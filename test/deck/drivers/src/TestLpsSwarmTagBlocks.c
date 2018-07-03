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
#include "freertosMocksImplementation.h"

#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#include "mock_estimator_kalman.h"
#include "mock_freertosMocks.h"

// Test helper functions

#define DW1000_MAXIMUM_COUNT (uint64_t)( 1099511627775 ) //The maximum timestamp the DW1000 can return (40 bits: 2^40 - 1)

/**
 Create a neighbours dictionary. Used for avoiding code duplication
 */
static dict* testCreateNeighboursDictionary() {
  return hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10);
}

/**
 Create a tof dictionary. Used for avoiding code duplication
 */
static dict* testCreateTofDictionary() {
  return hashtable2_dict_new(dict_uint16_cmp, dict_uint16_hash, 10);
}

/**
 Add an entry to the neighbours dictionary
 */
static void testFillNeighboursDictionary(dict* dct, const locoId_t id, const neighbourData_t data) {
  neighbourData_t* result = getDataForNeighbour(dct, id, true);
  *result = data;
}

/**
 Add an entry to the tof dictionary
 */
static void testFillTofDictionary(dict* dct, const locoId_t id1, const locoId_t id2, const tofData_t data) {
  tofData_t* result = getTofDataBetween(dct, id1, id2, true);
  *result = data;
}

/**
 A function to find a pair inside the payload of a packet. Needed because when passing the dictionary data to an array, there order is not specified
 */
static payload_t* testFindDataInPayload(lpsSwarmPacket_t* packet, locoId_t id) {
  for (unsigned int i = 0; i < packet->header.payloadLength; i++) {
    payload_t* payload = (payload_t*)&packet->payload;
    if (payload[i].id == id) {
      return &payload[i];
    }
  }
  return NULL;
}

/**
 Adjust the bit size of the number
 */
static uint64_t adjustBitSize(const uint64_t value, const uint64_t mask) {
    return value & mask;
}

// Tests

void setUp(void) {
}

void tearDown(void) {
}

/**
 Adjust the bit size of the number
 */
void testGenerateId() {
  // Fixture
  // Mock dwGetSystemTimestamp
  dwTime_t initialValue = { .full = 12345 };
  dwGetSystemTimestamp_Expect(NULL, NULL);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  initRandomizationEngine(NULL);
  const locoId_t firstId = generateId();
  const uint8_t maxCount = 10;
  uint8_t i = 0;

  // Test
  for (; i <= maxCount; i++) {
    const locoId_t newId = generateId();
    if (firstId != newId) {
      break;
    }
  }

  TEST_ASSERT_TRUE(i < maxCount);
}

void testVerifySeqNr() {
  // Fixture
  uint8_t expectedSeqNr = 10;

  for (uint8_t i = 0; i <= 4; i++) {
    // Test
    const bool result = verifySeqNr(i, expectedSeqNr);

    // Assert
    if ((expectedSeqNr - 5) <= i && i < expectedSeqNr) {
      TEST_ASSERT_EQUAL(false, result);
    } else {
      TEST_ASSERT_EQUAL(true, result);
    }
  }
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

  // Mock dwGetSystemTimestamp
  dwGetSystemTimestamp_Expect(NULL, NULL);
  dwGetSystemTimestamp_IgnoreArg_dev();
  dwGetSystemTimestamp_IgnoreArg_time();
  dwGetSystemTimestamp_ReturnThruPtr_time(&initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(NULL);

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
  uint64_t expectedValue = 13778432; // Approximatelly: expectedValueWithZeroInitialValue + initialValue.full (counting the wrapping) = expectedValueWithZeroInitialValue / 2 = 13778176

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
  dict* dct = testCreateNeighboursDictionary();

  neighbourData_t* result1 = getDataForNeighbour(dct, 1, false);
  neighbourData_t* result2 = getDataForNeighbour(dct, 1, true);

  TEST_ASSERT_EQUAL_PTR(NULL, result1);
  TEST_ASSERT_EQUAL_UINT64(0, result2->localRx);
  TEST_ASSERT_EQUAL_UINT64(0, result2->remoteTx);
  TEST_ASSERT_EQUAL_DOUBLE(1, result2->clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(0, result2->clockCorrectionStorage.clockCorrectionBucket);
  TEST_ASSERT_EQUAL_UINT8(0, result2->expectedSeqNr);
}

void testGetDataForNeighbourWithFilledDictionary() {
  dict* dct = testCreateNeighboursDictionary();
  neighbourData_t initialData = {
    .localRx = 1,
    .remoteTx = 2,
    .clockCorrectionStorage = {
      .clockCorrection = 3.47,
      .clockCorrectionBucket = 2
    },
    .expectedSeqNr = 123
  };

  testFillNeighboursDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 1, false);

  TEST_ASSERT_EQUAL_UINT64(initialData.localRx, result->localRx);
  TEST_ASSERT_EQUAL_UINT64(initialData.remoteTx, result->remoteTx);
  TEST_ASSERT_EQUAL_DOUBLE(initialData.clockCorrectionStorage.clockCorrection, result->clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(initialData.clockCorrectionStorage.clockCorrectionBucket, result->clockCorrectionStorage.clockCorrectionBucket);
  TEST_ASSERT_EQUAL_UINT8(initialData.expectedSeqNr, result->expectedSeqNr);
  TEST_ASSERT_NOT_EQUAL(&initialData, result);
}

void testGetDataForNeighbourWithFilledDictionaryAndDifferentKeys() {
  dict* dct = testCreateNeighboursDictionary();
  neighbourData_t initialData = {
    .localRx = 1,
    .remoteTx = 2,
    .clockCorrectionStorage = {
      .clockCorrection = 3.47,
      .clockCorrectionBucket = 2
    }
  };

  testFillNeighboursDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 2, false);

  TEST_ASSERT_EQUAL_PTR(NULL, result);
}

void testGetDataForNeighbourAndCount() {
  // Fixture
  dict* dct = testCreateNeighboursDictionary();
  uint8_t numberOfNeighbours = 123;

  // Test
  for (uint8_t i = 0; i < numberOfNeighbours; i++) {
    getDataForNeighbour(dct, i, true);
  }

  // Assert
  uint8_t count = dict_count(dct);
  TEST_ASSERT_EQUAL_UINT8(numberOfNeighbours, count);
}

void testSetTxDataWithoutPayload() {
  // Fixture
  dict* neighbourDct = testCreateNeighboursDictionary();
  dict* tofDct = testCreateTofDictionary();
  lpsSwarmPacket_t txPacket;
  locoId_t sourceId = 6;
  uint8_t nextSeqNr = 0;

  // Test
  setTxData(&txPacket, sourceId, &nextSeqNr, neighbourDct, tofDct);
  unsigned int txPacketLength = calculatePacketSize(&txPacket);

  // Assert
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacketHeader_t);
  locoId_t expectedSourceId = sourceId;
  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(expectedSourceId, txPacket.header.sourceId);
  TEST_ASSERT_EQUAL_UINT64(0, txPacket.header.tx);
  TEST_ASSERT_EQUAL_UINT8(0, txPacket.header.payloadLength);
  TEST_ASSERT_EQUAL_UINT8(0, txPacket.header.seqNr);
  TEST_ASSERT_EQUAL_UINT8(1, nextSeqNr);
}

void testSetTxDataWithPayload() {
  // Fixture
  dict* neighbourDct = testCreateNeighboursDictionary();
  dict* tofDct = testCreateTofDictionary();
  lpsSwarmPacket_t txPacket;
  locoId_t sourceId = 6;
  uint8_t nextSeqNr = 0;

  // Fill dictionary
  uint8_t elementsCount = 5;
  for (uint8_t i = 0; i < elementsCount; i++) {
    neighbourData_t neighbourData = { .localRx = i, .remoteTx = i + 1 };
    testFillNeighboursDictionary(neighbourDct, i, neighbourData);

    tofData_t tofData = { .tof = i + 2 };
    testFillTofDictionary(tofDct, sourceId, i, tofData);
  }

  // Test
  setTxData(&txPacket, sourceId, &nextSeqNr, neighbourDct, tofDct);
  unsigned int txPacketLength = calculatePacketSize(&txPacket);

  // Assert
  uint8_t expectedPayloadLength = elementsCount;
  unsigned int expectedTxPacketLength = sizeof(lpsSwarmPacketHeader_t) + expectedPayloadLength * sizeof(payload_t);
  locoId_t expectedSourceId = sourceId;
  TEST_ASSERT_EQUAL_UINT(expectedTxPacketLength, txPacketLength);
  TEST_ASSERT_EQUAL_UINT8(expectedSourceId, txPacket.header.sourceId);
  TEST_ASSERT_EQUAL_UINT64(0, txPacket.header.tx);
  TEST_ASSERT_EQUAL_UINT8(expectedPayloadLength, txPacket.header.payloadLength);
  for (uint8_t i = 0; i < expectedPayloadLength; i++) {
    payload_t* data = testFindDataInPayload(&txPacket, i);
    TEST_ASSERT_EQUAL_UINT8(i, data->id);
    TEST_ASSERT_EQUAL_UINT64(i, data->rx);
    TEST_ASSERT_EQUAL_UINT64(i + 1, data->tx);
    TEST_ASSERT_EQUAL_UINT16(i + 2, data->tof);
  }
  TEST_ASSERT_EQUAL_UINT8(0, txPacket.header.seqNr);
  TEST_ASSERT_EQUAL_UINT8(1, nextSeqNr);
}

void testProcessRxPacketWithPayload() {
  // Event timestamps:
  // * prevRemoteTx                                 ---> prevlocalRx = (clockCorrection * prevRemoteTx) + tof
  // * remoteRx = (localTx + tof) / clockCorrection <--- localTx = prevlocalRx + localReply
  // * remoteTx = remoteRx + remoteReply            ---> localRx = (clockCorrection * remoteTx) + tof

  // Expected values: giving them pair values we ensure proper integer operations (mainly division) when calculating the actual values
  uint32_t tof = 32640;                                 // Tof meassured by local clock
  double clockCorrection = 1.0 + 10e-6;                 // Local clock works <clockCorrection> times faster than remote clock
  uint32_t localReply = 400000;                         // Local reply time meassured by local clock. Any number.
  uint32_t remoteReply = 500000;                        // Remote reply time meassured by Remote clock. Any number.

  // Timestamp values
  uint64_t mask = 0xFFFFFFFFFF;
  uint64_t prevRemoteTx = adjustBitSize(mask - 300000, mask);   // Any number
  uint64_t prevLocalRx = adjustBitSize(mask - 50000, mask);     // Any number
  uint64_t localTx = adjustBitSize(prevLocalRx + localReply, mask);
  uint64_t remoteRx = adjustBitSize(prevRemoteTx + ((localReply + 2 * tof) / clockCorrection), mask);
  uint64_t remoteTx = adjustBitSize(remoteRx + remoteReply, mask);
  uint64_t localRx = adjustBitSize(localTx + ((clockCorrection * remoteReply) + 2 * tof), mask);

  //Ids
  locoId_t remoteId = 5;
  locoId_t localId = 6;

  // Create the neighbours dictionary
  dict* neighboursDct = testCreateNeighboursDictionary();

  // Set previous neighbour data
  neighbourData_t* prevNeighbourData = getDataForNeighbour(neighboursDct, remoteId, true);
  prevNeighbourData->remoteTx = prevRemoteTx;
  prevNeighbourData->localRx = prevLocalRx;
  prevNeighbourData->clockCorrectionStorage.clockCorrection = 5; // A wrong value on pourpose, so the proper clock correction is set and then we can assert it
  prevNeighbourData->clockCorrectionStorage.clockCorrectionBucket = 0;

  // Create the tof dictionary
  dict* tofDct = testCreateTofDictionary();

  // Setup the mock in charge of generating localRx
  dwTime_t localRxTimeStamp = { .full = localRx };
  dwGetReceiveTimestamp_Expect(NULL, NULL);
  dwGetReceiveTimestamp_IgnoreArg_dev();
  dwGetReceiveTimestamp_IgnoreArg_time();
  dwGetReceiveTimestamp_ReturnThruPtr_time(&localRxTimeStamp);

  // Create and fill rxPacket
  lpsSwarmPacket_t rxPacket;
  rxPacket.header.sourceId = remoteId;
  rxPacket.header.tx = remoteTx;
  rxPacket.header.payloadLength = 1;
  payload_t pair = {
    .id = localId,
    .rx = remoteRx,
    .tx = localTx
  };
  payload_t* payload = (payload_t*)&rxPacket.payload;
  payload[0] = pair;

  // Mock kalman filter
  estimatorKalmanEnqueueDistance_IgnoreAndReturn(true);
  // Mock tick count
  uint32_t tickCount = 12345;
  xTaskGetTickCount_IgnoreAndReturn(tickCount);

  // Test
  processRxPacket(NULL, localId, &rxPacket, neighboursDct, tofDct);

  // Verify result
  neighbourData_t* currentNeighbourData = getDataForNeighbour(neighboursDct, remoteId, false);
  tofData_t* currentTofData = getTofDataBetween(tofDct, localId, remoteId, false);

  TEST_ASSERT_EQUAL_UINT64(remoteTx, currentNeighbourData->remoteTx);
  TEST_ASSERT_EQUAL_UINT64(localRx, currentNeighbourData->localRx);
  TEST_ASSERT_EQUAL_UINT16(tof, currentTofData->tof); // It may be that -1 is required, because in the conversion between double and integer the system cuts the number down
  TEST_ASSERT_DOUBLE_WITHIN(10e-6, clockCorrection, currentNeighbourData->clockCorrectionStorage.clockCorrection); // Calculations make the clock have a slightly different value than the expected one. Using the whithin assertion mitigates this issue
  TEST_ASSERT_EQUAL_UINT(0, currentNeighbourData->clockCorrectionStorage.clockCorrectionBucket);
  TEST_ASSERT_EQUAL_UINT32(tickCount, currentNeighbourData->position.timestamp);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.x);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.y);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.z);
}

void testProcessRxPacketWithoutPayload() {
  // Preparation
  uint64_t mask = 0xFFFFFFFFFF;
  double clockCorrection = 1.0 + 10e-6;
  uint32_t localRxDifference = 460222;

  // Timestamp values
  uint64_t prevLocalRx = adjustBitSize(4500, mask);
  uint64_t localRx = adjustBitSize(prevLocalRx + localRxDifference, mask);
  uint64_t prevRemoteTx = adjustBitSize(20800, mask);
  uint64_t remoteTx =  adjustBitSize(prevRemoteTx + localRxDifference / clockCorrection, mask);

  // Other parameters
  locoId_t remoteId = 5;
  locoId_t localId = 6;
  uint8_t expectedSeqNr = 0;

  // Create the dictionary
  dict* neighbourDct = testCreateNeighboursDictionary();

  // Set previous data
  neighbourData_t* prevNeighbourData = getDataForNeighbour(neighbourDct, remoteId, true);
  prevNeighbourData->expectedSeqNr = expectedSeqNr;
  prevNeighbourData->remoteTx = prevRemoteTx;
  prevNeighbourData->localRx = prevLocalRx;
  prevNeighbourData->clockCorrectionStorage.clockCorrection = 5; // A wrong value on pourpose, so the proper clock correction is set and then we can assert it
  prevNeighbourData->clockCorrectionStorage.clockCorrectionBucket = 0;

  // Create the tof dictionary
  dict* tofDct = testCreateTofDictionary();

  // Setup the mock in charge of generating localRx
  dwTime_t localRxTimeStamp = { .full = localRx };
  dwGetReceiveTimestamp_Expect(NULL, NULL);
  dwGetReceiveTimestamp_IgnoreArg_dev();
  dwGetReceiveTimestamp_IgnoreArg_time();
  dwGetReceiveTimestamp_ReturnThruPtr_time(&localRxTimeStamp);

  // Create and fill rxPacket
  lpsSwarmPacket_t rxPacket;
  rxPacket.header.sourceId = remoteId;
  rxPacket.header.seqNr = expectedSeqNr;
  rxPacket.header.tx = remoteTx;
  rxPacket.header.payloadLength = 0;

  // Mock tick count
  uint32_t tickCount = 12345;
  xTaskGetTickCount_IgnoreAndReturn(tickCount);

  // Test
  processRxPacket(NULL, localId, &rxPacket, neighbourDct, tofDct);

  // Verify result
  neighbourData_t* currentNeighbourData = getDataForNeighbour(neighbourDct, remoteId, false);
  TEST_ASSERT_EQUAL_UINT64(remoteTx, currentNeighbourData->remoteTx);
  TEST_ASSERT_EQUAL_UINT64(localRx, currentNeighbourData->localRx);
  TEST_ASSERT_DOUBLE_WITHIN(10e-6, clockCorrection, currentNeighbourData->clockCorrectionStorage.clockCorrection); // Calculations make the clock have a slightly different value than the expected one. Using the whithin assertion mitigates this issue
  TEST_ASSERT_EQUAL_UINT8(expectedSeqNr + 1, currentNeighbourData->expectedSeqNr);
  TEST_ASSERT_EQUAL_UINT32(tickCount, currentNeighbourData->position.timestamp);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.x);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.y);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.z);
}

void testupdatePositionOfFirstDrone() {
  locoId_t remoteId = 5;
  locoId_t localId = 6;
  uint16_t tof = 12345;

  // Mock tick count
  uint32_t tickCount = 12345;
  xTaskGetTickCount_IgnoreAndReturn(tickCount);

  // Create the dictionaries
  dict* neighbourDct = testCreateNeighboursDictionary();
  dict* tofDct = testCreateTofDictionary();

  // Set state as if the data from drone has been already processed
  tofData_t* tofData = getTofDataBetween(tofDct, localId, remoteId, true);
  tofData->tof = tof;

  // Get data object for drone
  neighbourData_t* neighbourData = getDataForNeighbour(neighbourDct, remoteId, true);

  // Test
  updatePositionOf(localId, remoteId, neighbourData, neighbourDct, tofDct);

  // Assert
  neighbourData_t* currentNeighbourData = getDataForNeighbour(neighbourDct, remoteId, false);
  TEST_ASSERT_EQUAL_UINT32(tickCount, currentNeighbourData->position.timestamp);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.x);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.y);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.z);
}

void testupdatePositionOfSecondDroneWhenDataNotReady() {
  locoId_t remoteId1 = 5;
  locoId_t remoteId2 = 6;
  locoId_t localId = 7;
  locoId_t remoteIdOther = 8;

  uint16_t tofLocalToRemote1 = 12345;
  uint16_t tofLocalToRemote2 = 23456;
  uint16_t tofRemoteOtherToRemote2 = 34567;

  // Mock tick count
  uint32_t tickCount = 12345;
  xTaskGetTickCount_IgnoreAndReturn(tickCount);

  // Create the dictionaries
  dict* neighbourDct = testCreateNeighboursDictionary();
  dict* tofDct = testCreateTofDictionary();

  // Set state as if the data from drone 1 has been already processed
  {
    neighbourData_t* neighbourData = getDataForNeighbour(neighbourDct, remoteId1, true);
    neighbourData->position.timestamp = 123;
    neighbourData->position.x = 0;
    neighbourData->position.y = 0;
    neighbourData->position.z = 0;

    tofData_t* tofData = getTofDataBetween(tofDct, localId, remoteId1, true);
    tofData->tof = tofLocalToRemote1;
  }

  // Set state as if the data from drone 2 has been already processed
  {
    tofData_t* tofData = getTofDataBetween(tofDct, localId, remoteId2, true);
    tofData->tof = tofLocalToRemote2;
  }
  {
    tofData_t* tofData = getTofDataBetween(tofDct, remoteId2, remoteIdOther, true);
    tofData->tof = tofRemoteOtherToRemote2;
  }

  // Get data object for drone 2
  neighbourData_t* neighbourData2 = getDataForNeighbour(neighbourDct, remoteId2, true);

  // Test
  updatePositionOf(localId, remoteId2, neighbourData2, neighbourDct, tofDct);

  // Assert
  neighbourData_t* currentNeighbourData = getDataForNeighbour(neighbourDct, remoteId2, false);
  TEST_ASSERT_EQUAL_UINT32(0, currentNeighbourData->position.timestamp);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.x);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.y);
  TEST_ASSERT_EQUAL_FLOAT(0, currentNeighbourData->position.z);
}

void testupdatePositionOfWith2Drones() {
  locoId_t localId = 1;
  locoId_t remoteId1 = 2;
  locoId_t remoteId2 = 3;

  // Mock tick count
  uint32_t tickCount = 12345;
  xTaskGetTickCount_IgnoreAndReturn(tickCount);

  // Create the dictionaries
  dict* neighbourDct = testCreateNeighboursDictionary();
  dict* tofDct = testCreateTofDictionary();

  {
    locoId_t dutId = remoteId1;
    uint16_t tofDutToLocal = 1;

    {
      // Set state as if the data from drone has been already processed
      tofData_t* tofData = getTofDataBetween(tofDct, localId, dutId, true);
      tofData->tof = tofDutToLocal;

      // Get data object for drone
      neighbourData_t* neighbourData = getDataForNeighbour(neighbourDct, dutId, true);

      // Test
      updatePositionOf(localId, remoteId1, neighbourData, neighbourDct, tofDct);
    }

    // Assert
    neighbourData_t* dutData = getDataForNeighbour(neighbourDct, dutId, false);

    TEST_ASSERT_EQUAL_UINT32(tickCount, dutData->position.timestamp);
    TEST_ASSERT_EQUAL_FLOAT(0, dutData->position.x);
    TEST_ASSERT_EQUAL_FLOAT(0, dutData->position.y);
    TEST_ASSERT_EQUAL_FLOAT(0, dutData->position.z);
  }

  locoId_t dutId = remoteId2;
  locoId_t remoteId = remoteId1;

  for (unsigned int i = 0; i < 10; i++) {
    uint16_t tofDutToLocal = i * 2;
    uint16_t tofDutToRemote = i * 2 + 1;

    {
      // Set state as if the data from drone has been already processed
      {
        tofData_t* tofData = getTofDataBetween(tofDct, dutId, localId, true);
        tofData->tof = tofDutToLocal;
      }
      {
        tofData_t* tofData = getTofDataBetween(tofDct, dutId, remoteId, true);
        tofData->tof = tofDutToRemote;
      }

      // Get data object for drone
      neighbourData_t* dutData = getDataForNeighbour(neighbourDct, dutId, true);

      // Test
      updatePositionOf(localId, dutId, dutData, neighbourDct, tofDct);
    }

    // Assert
    neighbourData_t* dutData = getDataForNeighbour(neighbourDct, dutId, false);
    neighbourData_t* remoteData = getDataForNeighbour(neighbourDct, remoteId, false);
    float expectedXPosition = remoteData->position.x;
    if(dutId > remoteId) {
      expectedXPosition += (float)tofDutToRemote;
    } else {
      expectedXPosition -= (float)tofDutToRemote;
    }

    TEST_ASSERT_EQUAL_UINT32(tickCount, dutData->position.timestamp);
    TEST_ASSERT_EQUAL_FLOAT(expectedXPosition, dutData->position.x);
    TEST_ASSERT_EQUAL_FLOAT(0, dutData->position.y);
    TEST_ASSERT_EQUAL_FLOAT(0, dutData->position.z);

    printf("Data from drone %02d arrived | X position: %03g | tof to remote drone: %03d\n", dutId, dutData->position.x, tofDutToRemote);

    // Prepare for next iteration
    locoId_t oldDutId = dutId;
    locoId_t oldRemoteId = remoteId;
    dutId = oldRemoteId;
    remoteId = oldDutId;
  }
}
