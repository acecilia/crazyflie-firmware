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

#include "dw1000Mocks.h"
#include "freertosMocks.h"

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
  dwTime_t initialValue = { .full = 0 };
  dwTime_t expectedValue = { .full = 27556352 };

  dwDevice_t dev;
  dwGetSystemTimestamp_Expect(&dev, &initialValue);

  dwTime_t time = findTransmitTimeAsSoonAsPossible(&dev);
  
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

/**
 Add an entry to the dictionary
 */
static void fillDictionary(dict* dct, locoAddress_t address, neighbourData_t data) {
  neighbourData_t* result = getDataForNeighbour(dct, address);
  *result = data;
}

void testGetDataForNeighbourWithEmptyDictionary() {
  dict* dct = hashtable2_dict_new(dict_uint64_cmp, dict_uint64_hash, 10);

  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT32(0, result->localRx);
  TEST_ASSERT_EQUAL_UINT32(0, result->remoteTx);
  TEST_ASSERT_EQUAL_UINT32(0, result->tof);
}

void testGetDataForNeighbourWithFilledDictionary() {
  dict* dct = hashtable2_dict_new(dict_uint64_cmp, dict_uint64_hash, 10);
  neighbourData_t initialData = { .localRx = 1, .remoteTx = 2, .tof = 3 };

  fillDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 1);

  TEST_ASSERT_EQUAL_UINT32(initialData.localRx, result->localRx);
  TEST_ASSERT_EQUAL_UINT32(initialData.remoteTx, result->remoteTx);
  TEST_ASSERT_EQUAL_UINT32(initialData.tof, result->tof);
  TEST_ASSERT_NOT_EQUAL(&initialData, result);
}

void testGetDataForNeighbourWithFilledDictionaryAndDifferentKeys() {
  dict* dct = hashtable2_dict_new(dict_uint64_cmp, dict_uint64_hash, 10);
  neighbourData_t initialData = { .localRx = 1, .remoteTx = 2, .tof = 3 };

  fillDictionary(dct, 1, initialData);
  neighbourData_t* result = getDataForNeighbour(dct, 2);

  TEST_ASSERT_NOT_EQUAL(initialData.localRx, result->localRx);
  TEST_ASSERT_NOT_EQUAL(initialData.remoteTx, result->remoteTx);
  TEST_ASSERT_NOT_EQUAL(initialData.tof, result->tof);
  TEST_ASSERT_NOT_EQUAL(&initialData, result);
}
