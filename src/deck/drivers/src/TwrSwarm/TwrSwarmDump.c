#include "TwrSwarmDump.h"

#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "cfassert.h"
#include "log.h"
#include "limits.h"

#include "sysload.h"

static void timerHandler(xTimerHandle timer);

typedef struct {
  uint32_t ulRunTimeCounter;
  uint32_t xTaskNumber;
} taskData_t;

#define TASK_MAX_COUNT 16
bool initialized = false;
static taskData_t previousSnapshot[TASK_MAX_COUNT];
static int taskTopIndex = 0;
static uint32_t previousTotalRunTime = 0;

// Logging
static uint8_t load = 0;
static uint8_t idleLoad = 0;

static uint8_t lpsLoad = 0;
static uint16_t lpsStack = 0;

static uint16_t minStack = INT16_MAX;

void twrSwarmDumpInit() {
  ASSERT(!initialized);

  xTimerHandle timer = xTimerCreate( "twrSwarmDumpTimer", M2T(1000), pdTRUE, NULL, timerHandler);
  xTimerStart(timer, 1000);

  initialized = true;
}


static taskData_t* getPreviousTaskData(uint32_t xTaskNumber) {
  // Try to find the task in the list of tasks
  for (int i = 0; i < taskTopIndex; i++) {
    if (previousSnapshot[i].xTaskNumber == xTaskNumber) {
      return &previousSnapshot[i];
    }
  }

  // Allocate a new entry
  ASSERT(taskTopIndex < TASK_MAX_COUNT);
  taskData_t* result = &previousSnapshot[taskTopIndex];
  result->xTaskNumber = xTaskNumber;

  taskTopIndex++;

  return result;
}

static void timerHandler(xTimerHandle timer) {
  uint32_t totalRunTime;

  TaskStatus_t taskStats[TASK_MAX_COUNT];
  uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
  ASSERT(taskCount < TASK_MAX_COUNT);

  uint32_t totalDelta = totalRunTime - previousTotalRunTime;
  float f = 100.0 / totalDelta;

  // Dumps the the CPU load and stack usage for all tasks
  // CPU usage is since last dump in % compared to total time spent in tasks. Note that time spent in interrupts will be included in measured time.
  // Stack usage is displayed as nr of unused bytes at peak stack usage.

  uint8_t loadAcc = 0;
  for (uint32_t i = 0; i < taskCount; i++) {
    TaskStatus_t* stats = &taskStats[i];
    taskData_t* previousTaskData = getPreviousTaskData(stats->xTaskNumber);
    uint32_t taskRunTime = stats->ulRunTimeCounter;
    uint8_t taskLoad = (uint8_t)(f * (taskRunTime - previousTaskData->ulRunTimeCounter));
    uint16_t taskStack = stats->usStackHighWaterMark;

    if(strcmp(stats->pcTaskName, "lps") == 0) {
      // Do nothing
      lpsLoad = taskLoad;
      lpsStack = taskStack;
    }

    if(strcmp(stats->pcTaskName, "IDLE") == 0) {
      // Do nothing
      idleLoad = 100 - taskLoad;
    } else {
      loadAcc += taskLoad;

      if(taskStack < minStack) {
        DEBUG_PRINT("New minimum stack: %d | Set by: %s\n", taskStack, stats->pcTaskName);
        minStack = taskStack;
      }
    }

    previousTaskData->ulRunTimeCounter = taskRunTime;
    previousTotalRunTime = totalRunTime;
  }
  load = loadAcc;
}

LOG_GROUP_START(twrSwarmDump)
LOG_ADD(LOG_UINT8, load, &load)
LOG_ADD(LOG_UINT8, idleLoad, &idleLoad)
LOG_ADD(LOG_UINT8, lpsLoad, &lpsLoad)
LOG_ADD(LOG_UINT16, lpsStack, &lpsStack)
LOG_ADD(LOG_UINT16, minStack, &minStack)
LOG_GROUP_STOP(twrSwarmDump)
