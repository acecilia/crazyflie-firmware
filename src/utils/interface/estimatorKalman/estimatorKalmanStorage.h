#ifndef estimatorKalmanStorage_h
#define estimatorKalmanStorage_h

#include "FreeRTOS.h"
#include "queue.h"
#include "arm_math.h"

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t;

/*
 * The struct keeping the internal storage for the kalman estimator. Declare it statically: using it in any other way may cause stack overflow or fill the available dynamic memory
 */
typedef struct {
  float S[STATE_DIM];

  // The queues to add data to the filter
  xQueueHandle accelerationDataQueue;
  xQueueHandle angularVelocityDataQueue;
  xQueueHandle positionDataQueue;
  xQueueHandle distanceDataQueue;
  xQueueHandle velocityDataQueue;

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  float q[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // The covariance matrix
  float P[STATE_DIM][STATE_DIM];
  arm_matrix_instance_f32 Pm;

  // Internal variables
  bool isInit;
  uint32_t lastUpdate;
} estimatorKalmanStorage_t;

#endif /* estimatorKalmanStorage_h */
