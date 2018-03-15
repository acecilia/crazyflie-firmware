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
#include "lpsTdma.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Additions
#include "led.h"
#include "timers.h"
#include "debug.h"

// Outlier rejection
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4

// Rangin statistics
static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values
static uint8_t succededRanging[LOCODECK_NR_OF_ANCHORS];
static uint8_t failedRanging[LOCODECK_NR_OF_ANCHORS];

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

static lpsAlgoOptions_t* options;

// TDMA handling
static bool tdmaSynchronized;

static bool rangingOk;

// Additions
static int blinkCounter = 0;
static int rxPackagesCounter = 0;
static int lostPackagesCounter = 0;

static void blink(led_t led) {
  blinkCounter++;

  if (blinkCounter == 1000) {
    blinkCounter = 0;
    ledToggle(led);
  }
}

static xTimerHandle timer;
static void timerCallback(xTimerHandle timer) {
  DEBUG_PRINT("Received packages: %d | Lost packages: %d\n", rxPackagesCounter, lostPackagesCounter);
  rxPackagesCounter = 0;
  lostPackagesCounter = 0;
}

static void txcallback(dwDevice_t *dev) { }

static uint32_t rxcallback(dwDevice_t *dev) {
  blink(LED_RED_L);
  rxPackagesCounter++;

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;
  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[current_anchor];

  dwNewTransmit(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev)
{
  lostPackagesCounter++;
  blink(LED_BLUE_L);

  dwNewTransmit(dev);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  return;

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

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  dwIdle(dev);

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_LPP_SHORT;
  memcpy(&txPacket.payload[LPS_TWR_SEND_LPP_PAYLOAD], packet->data, packet->length);

  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[packet->dest];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  static uint32_t statisticStartTick = 0;

  if (statisticStartTick == 0) {
    statisticStartTick = xTaskGetTickCount();
  }

  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);

      if (lpp_transaction) {
        return 0;
      }
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      if (!ranging_complete && !lpp_transaction) {
        options->rangingState &= ~(1<<current_anchor);
        if (options->failedRanging[current_anchor] < options->rangingFailedThreshold) {
          options->failedRanging[current_anchor] ++;
          options->rangingState |= (1<<current_anchor);
        }

        locSrvSendRangeFloat(current_anchor, NAN);
        failedRanging[current_anchor]++;
      } else {
        options->rangingState |= (1<<current_anchor);
        options->failedRanging[current_anchor] = 0;

        locSrvSendRangeFloat(current_anchor, options->distance[current_anchor]);
        succededRanging[current_anchor]++;
      }

      // Handle ranging statistic
      if (xTaskGetTickCount() > (statisticStartTick+1000)) {
        statisticStartTick = xTaskGetTickCount();

        for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
          rangingPerSec[i] = failedRanging[i] + succededRanging[i];
          if (rangingPerSec[i] > 0) {
            rangingSuccessRate[i] = 100.0f*(float)succededRanging[i] / (float)rangingPerSec[i];
          } else {
            rangingSuccessRate[i] = 0.0f;
          }

          failedRanging[i] = 0;
          succededRanging[i] = 0;
        }
      }


      if (lpsGetLppShort(&lppShortPacket)) {
        lpp_transaction = true;
        sendLppShort(dev, &lppShortPacket);
      } else {
        lpp_transaction = false;
        ranging_complete = false;
        initiateRanging(dev);
      }
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
  timer = xTimerCreate("jejeTimer", M2T(5000), pdTRUE, NULL, timerCallback);
  xTimerStart(timer, 0);

  options = algoOptions;

  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

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

  memset(options->distance, 0, sizeof(options->distance));
  memset(options->pressures, 0, sizeof(options->pressures));
  memset(options->failedRanging, 0, sizeof(options->failedRanging));

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

LOG_GROUP_START(twr)
LOG_ADD(LOG_UINT8, rangingSuccessRate0, &rangingSuccessRate[0])
LOG_ADD(LOG_UINT8, rangingPerSec0, &rangingPerSec[0])
LOG_ADD(LOG_UINT8, rangingSuccessRate1, &rangingSuccessRate[1])
LOG_ADD(LOG_UINT8, rangingPerSec1, &rangingPerSec[1])
LOG_ADD(LOG_UINT8, rangingSuccessRate2, &rangingSuccessRate[2])
LOG_ADD(LOG_UINT8, rangingPerSec2, &rangingPerSec[2])
LOG_ADD(LOG_UINT8, rangingSuccessRate3, &rangingSuccessRate[3])
LOG_ADD(LOG_UINT8, rangingPerSec3, &rangingPerSec[3])
LOG_ADD(LOG_UINT8, rangingSuccessRate4, &rangingSuccessRate[4])
LOG_ADD(LOG_UINT8, rangingPerSec4, &rangingPerSec[4])
LOG_ADD(LOG_UINT8, rangingSuccessRate5, &rangingSuccessRate[5])
LOG_ADD(LOG_UINT8, rangingPerSec5, &rangingPerSec[5])
LOG_GROUP_STOP(twr)
