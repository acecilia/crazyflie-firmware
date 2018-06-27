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

static bool rangingOk;

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

  return twrSwarmAlgorithm.onEvent(dev, event);
}

static void twrTagInit(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions)
{
#ifdef LPS_TWR_SWARM_DEBUG_ENABLE
  debug.init();
#endif

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);
  dwCommitConfiguration(dev);
  rangingOk = false;

  twrSwarmAlgorithm.init(dev);
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
