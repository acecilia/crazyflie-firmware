#ifndef __LPS_TWR_SWARM_TAG_H__
#define __LPS_TWR_SWARM_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWR_LPP_SHORT 0xF0

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1
// LPP payload can be in the ANSWER packet
#define LPS_TWR_LPP_HEADER 2
#define LPS_TWR_LPP_TYPE 3
#define LPS_TWR_LPP_PAYLOAD 4

#define LPS_TWR_SEND_LPP_PAYLOAD 1

extern uwbAlgorithm_t uwbTwrSwarmTagAlgorithm;

#define TWR_RECEIVE_TIMEOUT 1000

#endif // __LPS_TWR_SWARM_TAG_H__
