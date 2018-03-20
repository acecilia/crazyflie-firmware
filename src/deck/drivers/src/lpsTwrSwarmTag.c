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


// Rangin statistics
static uint8_t rangingPerSec = 0;
static uint8_t rangingSuccessRate = 0;
// Used to calculate above values
static uint8_t succededRanging = 0;
static uint8_t failedRanging = 0;

/*
// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
*/

static packet_t txPacket;
/*
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

static lpsAlgoOptions_t* options;

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

static void txcallback(dwDevice_t *dev) { }

static uint32_t rxcallback(dwDevice_t *dev) {
  blink(LED_RED_L);
  succededRanging++;

  /*
  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;
  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[current_anchor];
   */

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  // dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev)
{
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

      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      /*
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
      */

      failedRanging++;

      if (xTaskGetTickCount() > (statisticStartTick+1000)) {
        statisticStartTick = xTaskGetTickCount();

        rangingPerSec = failedRanging + succededRanging;
        if (rangingPerSec > 0) {
          rangingSuccessRate = 100.0f*(float)succededRanging / (float)rangingPerSec;
        } else {
          rangingSuccessRate = 0.0f;
        }

        failedRanging = 0;
        succededRanging = 0;
      }

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
  // options = algoOptions;

  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

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

/*
LOG_GROUP_START(twrSwarm)
LOG_ADD(LOG_UINT8, rangingSuccessRate, &rangingSuccessRate)
LOG_ADD(LOG_UINT8, rangingPerSec, &rangingPerSec)
LOG_GROUP_STOP(twrSwarm)
*/
