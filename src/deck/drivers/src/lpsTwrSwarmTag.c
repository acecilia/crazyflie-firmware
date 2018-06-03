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

#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "arm_math.h"

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
#include "TwrSwarmDebug.h"
#endif

#include "TwrSwarmAlgorithm.h"

static lpsAlgoOptions_t* options;
static bool rangingOk;

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

  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev) {
  twrSwarmAlgorithm.initiateRanging(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.blink(LED_BLUE_L);

  switch(event) {
    case eventPacketReceived:
      debug.blink(LED_GREEN_L);
      break;
    case eventPacketSent:
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      debug.blink(LED_RED_L);
      break;
    case eventReceiveTimeout:
      debug.blink(LED_GREEN_R);
      break;
    case eventReceiveFailed:
      debug.blink(LED_GREEN_R);
      break;
    default:
      break;
  }
#endif

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

#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.init();
#endif

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
