#include "randomizedTimerFunctions.h"

#include "unity.h"

#include "timersMocks.h"

void setUp(void) {
}

void tearDown(void) {
}

void testSetFrequency() {
  // Fixture
  uint16_t frequency = 500;
  randomizedTimer_t randomizedTimer;
  
  // Test
  setFrequency(&randomizedTimer, frequency);

  // Assert
  const double expectedResult = 2; // TicksPerSecond are 1000
  TEST_ASSERT_EQUAL_UINT16(expectedResult, randomizedTimer.averagePeriod);
}

void testRandomizePeriod() {
  // Fixture
  uint32_t averagePeriod = 1000;

  for(uint32_t i = 0; i < 100; i++) {
    // Test
    uint32_t result = randomizePeriod(averagePeriod);

    // Assert
    TEST_ASSERT_GREATER_OR_EQUAL(averagePeriod / 2, result);
    TEST_ASSERT_LESS_OR_EQUAL(averagePeriod + averagePeriod / 2, result);
  }
}
