#include "randomizedTimerFunctions.h"

#include "unity.h"

#include "mock_timersMocks.h"

void setUp(void) {
}

void tearDown(void) {
}

void testSetFrequency() {
  // Fixture
  const uint16_t frequency = 500;
  randomizedTimer_t randomizedTimer;
  
  // Test
  setFrequency(&randomizedTimer, frequency);

  // Assert
  const double expectedResult = 2; // TicksPerSecond are 1000
  TEST_ASSERT_EQUAL_UINT16(expectedResult, randomizedTimer.averagePeriod);
}

void testRandomizePeriod() {
  // Fixture
  const uint32_t averagePeriod = 1000;

  for(uint32_t i = 0; i < 100; i++) {
    // Test
    const uint32_t result = randomizePeriod(averagePeriod);

    // Assert
    TEST_ASSERT_GREATER_OR_EQUAL(averagePeriod / 2, result);
    TEST_ASSERT_LESS_OR_EQUAL(averagePeriod + averagePeriod / 2, result);
  }
}

void testInit() {
  // Fixture
  const uint16_t frequency = 500;
  const xTimerHandle timer = (void*) 1234;
  void (*callback)(void) = (void (*)(void)) 5678;

  randomizedTimer_t randomizedTimer;
  xTimerCreate_IgnoreAndReturn(timer);

  // Test
  init(&randomizedTimer, frequency, callback);

  // Assert
  const double expectedPeriod = 2; // TicksPerSecond are 1000
  const xTimerHandle expectedTimer = timer;
  void (*expectedCallback)(void) = callback;
  TEST_ASSERT_EQUAL_UINT16(expectedPeriod, randomizedTimer.averagePeriod);
  TEST_ASSERT_EQUAL(expectedTimer, randomizedTimer.timer);
  TEST_ASSERT_EQUAL(expectedCallback, randomizedTimer.callback);
}
