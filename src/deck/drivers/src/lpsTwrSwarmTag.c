/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* lpsTwrSwarmTag.c: Uwb two way ranging anchor implementation for swarms */


#include <string.h>
#include <math.h>

#include "lpsTwrSwarmTag.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Additions
#include "led.h"
#include "debug.h"
#include "timers.h"

#include "TwrSwarmAlgorithm.h"

// Rangin statistics
static uint16_t rangingPerSec = 0;
static uint8_t performanceRate = 0;
// Used to calculate above values
static uint16_t succededRangingCounter = 0;
static uint16_t failedRangingCounter = 0;

static lpsAlgoOptions_t* options;
static bool rangingOk;

// Additions
static unsigned int BLUE_L_counter = 0;
static unsigned int GREEN_L_counter = 0;
static unsigned int RED_L_counter = 0;
static unsigned int GREEN_R_counter = 0;


static void blink(led_t led) {
  unsigned int* blinkCounter;

  switch (led) {
    case LED_BLUE_L:
      blinkCounter = &BLUE_L_counter;
      break;
    case LED_GREEN_L:
      blinkCounter = &GREEN_L_counter;
      break;
    case LED_RED_L:
      blinkCounter = &RED_L_counter;
      break;
    case LED_GREEN_R:
      blinkCounter = &GREEN_R_counter;
      break;
    default:
      configASSERT(false);
      break;
  }

  (*blinkCounter)++;

  if (*blinkCounter == 1000) {
    *blinkCounter = 0;
    ledToggle(led);
  }
}
static xTimerHandle logTimer;
static void logTimerCallback(xTimerHandle timer) {
  rangingPerSec = failedRangingCounter + succededRangingCounter;
  if (rangingPerSec > 0) {
    performanceRate = 100.0f*(float)succededRangingCounter / (float)rangingPerSec;
  } else {
    performanceRate = 0.0f;
  }

  failedRangingCounter = 0;
  succededRangingCounter = 0;
}

/*static void sendData(dwDevice_t *dev, void *data, bool waitForResponse) {
  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&data, MAC802154_HEADER_LENGTH+2);
  dwWaitForResponse(dev, waitForResponse);
  dwStartTransmit(dev);
}

static void waitForResponse(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}*/

static void txcallback(dwDevice_t *dev) {
  twrSwarmAlgorithm.txcallback(dev);
}

static uint32_t rxcallback(dwDevice_t *dev) {
  unsigned int dataLength = dwGetDataLength(dev);

  if (dataLength > 0) {
    lpsSwarmPacket_t* rxPacket = pvPortMalloc(dataLength);
    dwGetData(dev, (uint8_t*)rxPacket, dataLength);
    twrSwarmAlgorithm.rxcallback(dev, options, rxPacket, dataLength);
    vPortFree(rxPacket);
  }

  /*
  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  */

  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev) {
  twrSwarmAlgorithm.initiateRanging(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  blink(LED_BLUE_L);

  switch(event) {
    case eventPacketReceived:
      blink(LED_GREEN_L);
      break;
    case eventPacketSent:
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      blink(LED_RED_L);
      break;
    case eventReceiveTimeout:
      blink(LED_GREEN_R);
      break;
    case eventReceiveFailed:
      blink(LED_GREEN_R);
      break;
    default:
      break;
  }

  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      initiateRanging(dev); // Added
      return MAX_TIMEOUT;
      break;
    case eventReceiveTimeout:
    case eventReceiveFailed:
      return 0;
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions)
{
  twrSwarmAlgorithm.init();

  options = algoOptions;

  // Initialize the logging timer
  logTimer = xTimerCreate("loggingTimer", M2T(1000), pdTRUE, NULL, logTimerCallback);
  xTimerStart(logTimer, 0);

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);
  dwCommitConfiguration(dev);
  rangingOk = false;

  /*for(int i = 0; i < 5; i++) {
    DEBUG_PRINT("\n\nRxcallback:\n");

    ////////////
    // Set new values on the dictionary
    neighbourData_t* data = twrSwarmAlgorithm.getDataForNeighbour(twrSwarmAlgorithm.ctx->dct, 66);
    data->localRx = data->localRx + 10;
    data->remoteTx = 66;
    data->tof = 66;

    // Verify that the data is saved correctly
    neighbourData_t* neighbourData = twrSwarmAlgorithm.getDataForNeighbour(twrSwarmAlgorithm.ctx->dct, 66);
    if (neighbourData->localRx == data->localRx) {
      DEBUG_PRINT("Ok: Data saved correctly");
    } else {
      DEBUG_PRINT("Error: expected data not found in dictionary");
    }
    ////////////

    twrSwarmAlgorithm.rxcallback(dev, options, NULL, 0);
  }*/
}

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTwrSwarmTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
};

//LOG_GROUP_START(twrSwarm)
//LOG_ADD(LOG_UINT16, rangingPerSec, &rangingPerSec)
//LOG_ADD(LOG_UINT8, performanceRate, &performanceRate)
//LOG_GROUP_STOP(twrSwarm)
