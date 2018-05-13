/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sysload.c - System load monitor
 */

#define DEBUG_MODULE "SYSLOAD"

#include <stdbool.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "cfassert.h"
#include "param.h"
#include "libdict_test.h"

#include "sysload.h"

#define TIMER_PERIOD M2T(1000)

static void timerHandler(xTimerHandle timer);

static bool initialized = false;
static uint8_t triggerDump = 0;

typedef struct {
  uint32_t ulRunTimeCounter;
  uint32_t xTaskNumber;
} taskData_t;

#define TASK_MAX_COUNT 16
static taskData_t previousSnapshot[TASK_MAX_COUNT];
static int taskTopIndex = 0;
static uint32_t previousTotalRunTime = 0;

void sysLoadInit() {
  ASSERT(!initialized);

  xTimerHandle timer = xTimerCreate( "sysLoadMonitorTimer", TIMER_PERIOD, pdTRUE, NULL, timerHandler);
  xTimerStart(timer, 100);

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


static char* getName(dictionary_t type) {
  switch (type) {
    case height_balanced: return "height_balanced";
    case path_reduction: return "path_reduction";
    case red_black: return "red_black";
    case treap: return "treap";
    case splay: return "splay";
    case skip_list: return "skip_list";
    case weight_balanced: return "weight_balanced";
    case hashtable_separate_chaining: return "hashtable_separate_chaining";
    case hashtable_open_addressing: return "hashtable_open_addressing";
    default: return "error";
  }
}

static void test_libdict() {
  // Required to disable the watchdog (find watchdogInit in the project and comment it). Enabling the DEBUG mode brings memory problems
  dictionary_t type_array[] = {
    height_balanced,
    path_reduction,
    red_black,
    treap,
    splay,
    skip_list,
    weight_balanced,
    hashtable_separate_chaining,
    hashtable_open_addressing
  };
  int length = sizeof(type_array) / sizeof(dictionary_t);

  configure_dict_malloc();

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  DEBUG_PRINT("### Insert/search test:\n");
  for(int i = 0; i < length; i++) {
    char* r1 = test_uint8_insert_search(type_array[i]) ? "Ok" : "Failed";
    DEBUG_PRINT("%s [%s]\n", r1, getName(type_array[i]));
  }

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  int benchmark_duration_ms = 5000;
  DEBUG_PRINT("### Insert/remove benchmark (%d ms):\n", benchmark_duration_ms);
  for(int i = 0; i < length; i++) {
    int r1 = benchmark_uint_keys_dict_insert_remove(type_array[i], benchmark_duration_ms);
    DEBUG_PRINT("%d [%s]\n", r1, getName(type_array[i]));
  }

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());
}

static void timerHandler(xTimerHandle timer) {
  if (triggerDump != 0) {
    uint32_t totalRunTime;

    TaskStatus_t taskStats[TASK_MAX_COUNT];
    uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
    ASSERT(taskCount < TASK_MAX_COUNT);

    uint32_t totalDelta = totalRunTime - previousTotalRunTime;
    float f = 100.0 / totalDelta;

    // Dumps the the CPU load and stack usage for all tasks
    // CPU usage is since last dump in % compared to total time spent in tasks. Note that time spent in interrupts will be included in measured time.
    // Stack usage is displayed as nr of unused bytes at peak stack usage.

    DEBUG_PRINT("Task dump\n");
    DEBUG_PRINT("Load\tStack left\tName\n");
    for (uint32_t i = 0; i < taskCount; i++) {
      TaskStatus_t* stats = &taskStats[i];
      taskData_t* previousTaskData = getPreviousTaskData(stats->xTaskNumber);

      uint32_t taskRunTime = stats->ulRunTimeCounter;
      float load = f * (taskRunTime - previousTaskData->ulRunTimeCounter);
      DEBUG_PRINT("%.2f \t%u \t%s\n", (double)load, stats->usStackHighWaterMark, stats->pcTaskName);
      
      previousTaskData->ulRunTimeCounter = taskRunTime;
    }

    previousTotalRunTime = totalRunTime;

    test_libdict();

    triggerDump = 0;
  }
}

PARAM_GROUP_START(system)
PARAM_ADD(PARAM_UINT8, taskDump, &triggerDump)
PARAM_GROUP_STOP(system)
