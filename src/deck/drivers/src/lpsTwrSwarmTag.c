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

#define INDEX 1

/*
// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
*/

static lpsAlgoOptions_t* options;
/*
static packet_t txPacket;

static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

// TDMA handling
static bool tdmaSynchronized;
*/

static bool rangingOk;

// Additions
static int blinkCounter = 0;
static void blink(led_t led) {
  blinkCounter++;

  if (blinkCounter == 1000) {
    blinkCounter = 0;
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
  // dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return 0;

  lpsSwarmPacket_t rxPacket;
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  return twrSwarmAlgorithm.rxcallback(dev, &rxPacket, options);
}

static void initiateRanging(dwDevice_t *dev) {
  twrSwarmAlgorithm.initiateRanging(dev);
  
  /* Previous implementation, to use as a guide
  dwIdle(dev);

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;
  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[current_anchor];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  */
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  static uint32_t statisticStartTick = 0;

  if (statisticStartTick == 0) {
    statisticStartTick = xTaskGetTickCount();
  }

  switch(event) {
    case eventPacketReceived:
      blink(LED_RED_L);
      succededRangingCounter++;
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);

      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      failedRangingCounter++;
      blink(LED_BLUE_L);
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
  
  // Initialize the packet in the TX buffer
  // memset(&txPacket, 0, sizeof(txPacket));
  // MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  // txPacket.pan = 0xbccf;

  /*
  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  current_anchor = 0;

  options->rangingState = 0;
  ranging_complete = false;

  tdmaSynchronized = false;
  */

  memset(algoOptions->distance, 0, sizeof(algoOptions->distance));
  memset(algoOptions->pressures, 0, sizeof(algoOptions->pressures));
  memset(algoOptions->failedRanging, 0, sizeof(algoOptions->failedRanging));

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
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

LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_UINT16, rangingPerSec, &rangingPerSec)
LOG_ADD(LOG_UINT8, performanceRate, &performanceRate)
LOG_GROUP_STOP(twrSwarm)
