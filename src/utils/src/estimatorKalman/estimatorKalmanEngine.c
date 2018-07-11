#include "estimatorKalmanEngine.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "param.h"

#include "math.h"
#include "arm_math.h"


/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
// static void predict(kalmanStorage_t* storage, float thrust, Axis3f *acc, Axis3f *gyro, float dt);
// static void addProcessNoise(kalmanStorage_t* storage, float dt);

/*  - Measurement updates based on sensors */
// static void scalarUpdate(kalmanStorage_t* storage, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);
// static void stateEstimatorUpdateWithAccOnGround(Axis3f *acc);
// #ifdef KALMAN_USE_BARO_UPDATE
// static void stateEstimatorUpdateWithBaro(baro_t *baro);
// #endif

/*  - Finalization to incorporate attitude error into body attitude */
// static void finalize(kalmanStorage_t* storage, sensorData_t *sensors, uint32_t tick);

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
// static void stateEstimatorExternalizeState(kalmanStorage_t* storage, state_t *state, sensorData_t *sensors, uint32_t tick);

/**
 * Constants
 */

#define RAD_TO_DEG (180.0f/PI)

/**
 * Tuning parameters
 */

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// The bounds on states, these shouldn't be hit...
#define MAX_POSITION (100) //meters
#define MAX_VELOCITY (10) //meters per second

// Initial variances, uncertain of position, but know we're stationary and roughly flat
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;

static const float initialX = 0.5;
static const float initialY = 0.5;
static const float initialZ = 0.0;

/**
 * Queue related
 */

#define POS_QUEUE_LENGTH (10)
#define DIST_QUEUE_LENGTH (10)

static inline bool hasDistanceMeasurement(kalmanStorage_t* storage, distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(storage->distDataQueue, dist, 0));
}

static inline bool hasPositionMeasurement(kalmanStorage_t* storage, positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(storage->posDataQueue, pos, 0));
}

/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst));
}
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
}
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) {
  configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst));
}
static inline float arm_sqrt(float32_t in) {
  float pOut = 0;
  arm_status result = arm_sqrt_f32(in, &pOut);
  configASSERT(ARM_MATH_SUCCESS == result);
  return pOut;
}

/*
 * Other functions
 */
static void stateEstimatorAssertNotNaN() {
  return;
}

#ifdef KALMAN_DECOUPLE_XY
// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(kalmanStorage_t* storage, stateIdx_t state) {
  // Set all covariance to 0
  for(int i=0; i<STATE_DIM; i++) {
    storage->P[state][i] = 0;
    storage->P[i][state] = 0;
  }
  // Set state variance to maximum
  storage->P[state][state] = MAX_COVARIANCE;
  // set state to zero
  storage->S[state] = 0;
}
#endif

/*
 * Main Kalman Filter functions
 */

static void init(kalmanStorage_t* storage) {
  // Reset the queues
  if (!storage->isInit) {
    storage->posDataQueue = xQueueCreate(POS_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    storage->distDataQueue = xQueueCreate(DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
  } else {
    xQueueReset(storage->posDataQueue);
    xQueueReset(storage->distDataQueue);
  }

  /*
   storage->lastPrediction = xTaskGetTickCount();
   storage->lastPNUpdate = xTaskGetTickCount();
   */

  // Reset the state
  // TODO: Can we initialize this more intelligently?
  storage->S[STATE_X] = initialX;
  storage->S[STATE_Y] = initialY;
  storage->S[STATE_Z] = initialZ;
  storage->S[STATE_PX] = 0;
  storage->S[STATE_PY] = 0;
  storage->S[STATE_PZ] = 0;
  storage->S[STATE_D0] = 0;
  storage->S[STATE_D1] = 0;
  storage->S[STATE_D2] = 0;

  // reset the attitude quaternion
  for(int i=0; i<3; i++) {
    storage->q[i] = i == 0 ? 1 : 0;
  }

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      storage->R[i][j] = i==j ? 1 : 0;
    }
  }

  // Reset covariance matrix
  for (int i=0; i< STATE_DIM; i++) {
    for (int j=0; j < STATE_DIM; j++) {
      storage->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  storage->P[STATE_X][STATE_X] = powf(stdDevInitialPosition_xy, 2);
  storage->P[STATE_Y][STATE_Y] = powf(stdDevInitialPosition_xy, 2);
  storage->P[STATE_Z][STATE_Z] = powf(stdDevInitialPosition_z, 2);

  storage->P[STATE_PX][STATE_PX] = powf(stdDevInitialVelocity, 2);
  storage->P[STATE_PY][STATE_PY] = powf(stdDevInitialVelocity, 2);
  storage->P[STATE_PZ][STATE_PZ] = powf(stdDevInitialVelocity, 2);

  storage->P[STATE_D0][STATE_D0] = powf(stdDevInitialAttitude_rollpitch, 2);
  storage->P[STATE_D1][STATE_D1] = powf(stdDevInitialAttitude_rollpitch, 2);
  storage->P[STATE_D2][STATE_D2] = powf(stdDevInitialAttitude_yaw, 2);

  storage->Pm = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, (float *)storage->P };

  // Initialise tmp variables
  storage->Am = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, (float *)storage->A };
  storage->Km = (arm_matrix_instance_f32){ STATE_DIM, 1, storage->K };
  storage->tmpNN1m = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, storage->tmpNN1d };
  storage->tmpNN2m = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, storage->tmpNN2d };
  storage->tmpNN3m = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, storage->tmpNN3d };
  storage->HTm = (arm_matrix_instance_f32){ STATE_DIM, 1, storage->HTd };
  storage->PHTm = (arm_matrix_instance_f32){ STATE_DIM, 1, storage->PHTd };

  storage->isInit = true;
}

static void scalarUpdate(kalmanStorage_t* storage, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise) {
  configASSERT(Hm->numRows == 1);
  configASSERT(Hm->numCols == STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &storage->HTm);
  mat_mult(&storage->Pm, &storage->HTm, &storage->PHTm); // PH'
  float R = powf(stdMeasNoise, 2);
  float HPHR = R; // HPH' + R
  for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*storage->PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  configASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<STATE_DIM; i++) {
    storage->K[i] = storage->PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    storage->S[i] = storage->S[i] + storage->K[i] * error; // state update
  }
  stateEstimatorAssertNotNaN();

  // ====== COVARIANCE UPDATE ======
  mat_mult(&storage->Km, Hm, &storage->tmpNN1m); // KH
  for (int i=0; i<STATE_DIM; i++) {
    storage->tmpNN1d[STATE_DIM*i+i] -= 1;
  } // KH - I
  mat_trans(&storage->tmpNN1m, &storage->tmpNN2m); // (KH - I)'
  mat_mult(&storage->tmpNN1m, &storage->Pm, &storage->tmpNN3m); // (KH - I)*P
  mat_mult(&storage->tmpNN3m, &storage->tmpNN2m, &storage->Pm); // (KH - I)*P*(KH - I)'
  stateEstimatorAssertNotNaN();
  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float v = storage->K[i] * R * storage->K[j];
      float p = 0.5f*storage->P[i][j] + 0.5f*storage->P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        storage->P[i][j] = storage->P[j][i] = MAX_COVARIANCE;
      } else if (i==j && p < MIN_COVARIANCE) {
        storage->P[i][j] = storage->P[j][i] = MIN_COVARIANCE;
      } else {
        storage->P[i][j] = storage->P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}

static void updateWithPosition(kalmanStorage_t* storage, positionMeasurement_t *xyz) {
  // a direct measurement of states x, y, and z
  // do a scalar update for each state, since this should be faster than updating all together
  for (int i=0; i<3; i++) {
    if (!isnan(xyz->pos[i])) { // Only update positions that are valid: allows to update position components individually
      float h[STATE_DIM] = {0};
      arm_matrix_instance_f32 H = {1, STATE_DIM, h};
      h[STATE_X+i] = 1;
      scalarUpdate(storage, &H, xyz->pos[i] - storage->S[STATE_X+i], xyz->stdDev);
    }
  }
}

static void updateWithDistance(kalmanStorage_t* storage, distanceMeasurement_t *d)
{
  // a measurement of distance to point (x, y, z)
  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  float dx = storage->S[STATE_X] - d->x;
  float dy = storage->S[STATE_Y] - d->y;
  float dz = storage->S[STATE_Z] - d->z;

  float predictedDistance = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
  float measuredDistance = d->distance;

  // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The derivative dz/dX gives h.
  h[STATE_X] = dx/predictedDistance;
  h[STATE_Y] = dy/predictedDistance;
  h[STATE_Z] = dz/predictedDistance;

  scalarUpdate(storage, &H, measuredDistance-predictedDistance, d->stdDev);
}

static void finalize(kalmanStorage_t* storage, uint32_t tick) {
  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = storage->S[STATE_D0];
  float v1 = storage->S[STATE_D1];
  float v2 = storage->S[STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2);
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = dq[0] * storage->q[0] - dq[1] * storage->q[1] - dq[2] * storage->q[2] - dq[3] * storage->q[3];
    float tmpq1 = dq[1] * storage->q[0] + dq[0] * storage->q[1] + dq[3] * storage->q[2] - dq[2] * storage->q[3];
    float tmpq2 = dq[2] * storage->q[0] - dq[3] * storage->q[1] + dq[0] * storage->q[2] + dq[1] * storage->q[3];
    float tmpq3 = dq[3] * storage->q[0] + dq[2] * storage->q[1] - dq[1] * storage->q[2] + dq[0] * storage->q[3];

    // normalize and store the result
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
    storage->q[0] = tmpq0 / norm;
    storage->q[1] = tmpq1 / norm;
    storage->q[2] = tmpq2 / norm;
    storage->q[3] = tmpq3 / norm;

    /** Rotate the covariance, since we've rotated the body
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

    float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
    float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
    float d2 = v2/2;

    storage->A[STATE_X][STATE_X] = 1;
    storage->A[STATE_Y][STATE_Y] = 1;
    storage->A[STATE_Z][STATE_Z] = 1;

    storage->A[STATE_PX][STATE_PX] = 1;
    storage->A[STATE_PY][STATE_PY] = 1;
    storage->A[STATE_PZ][STATE_PZ] = 1;

    storage->A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    storage->A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
    storage->A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

    storage->A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
    storage->A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    storage->A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

    storage->A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
    storage->A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
    storage->A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&storage->Am, &storage->tmpNN1m); // A'
    mat_mult(&storage->Am, &storage->Pm, &storage->tmpNN2m); // AP
    mat_mult(&storage->tmpNN2m, &storage->tmpNN1m, &storage->Pm); //APA'
  }

  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  storage->R[0][0] = powf(storage->q[0], 2) + powf(storage->q[1], 2) - powf(storage->q[2], 2) - powf(storage->q[3], 2);
  storage->R[0][1] = 2 * storage->q[1] * storage->q[2] - 2 * storage->q[0] * storage->q[3];
  storage->R[0][2] = 2 * storage->q[1] * storage->q[3] + 2 * storage->q[0] * storage->q[2];

  storage->R[1][0] = 2 * storage->q[1] * storage->q[2] + 2 * storage->q[0] * storage->q[3];
  storage->R[1][1] = powf(storage->q[0], 2) - powf(storage->q[1], 2) + powf(storage->q[2], 2) - powf(storage->q[3], 2);
  storage->R[1][2] = 2 * storage->q[2] * storage->q[3] - 2 * storage->q[0] * storage->q[1];

  storage->R[2][0] = 2 * storage->q[1] * storage->q[3] - 2 * storage->q[0] * storage->q[2];
  storage->R[2][1] = 2 * storage->q[2] * storage->q[3] + 2 * storage->q[0] * storage->q[1];
  storage->R[2][2] = powf(storage->q[0], 2) - powf(storage->q[1], 2) - powf(storage->q[2], 2) + powf(storage->q[3], 2);

  // reset the attitude error
  storage->S[STATE_D0] = 0;
  storage->S[STATE_D1] = 0;
  storage->S[STATE_D2] = 0;

  // constrain the states
  for (int i=0; i<3; i++) {
    if (storage->S[STATE_X+i] < -MAX_POSITION) {
      storage->S[STATE_X+i] = -MAX_POSITION;
    } else if (storage->S[STATE_X+i] > MAX_POSITION) {
      storage->S[STATE_X+i] = MAX_POSITION;
    }

    if (storage->S[STATE_PX+i] < -MAX_VELOCITY) {
      storage->S[STATE_PX+i] = -MAX_VELOCITY;
    } else if (storage->S[STATE_PX+i] > MAX_VELOCITY) {
      storage->S[STATE_PX+i] = MAX_VELOCITY;
    }
  }

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*storage->P[i][j] + 0.5f*storage->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        storage->P[i][j] = storage->P[j][i] = MAX_COVARIANCE;
      } else if (i==j && p < MIN_COVARIANCE) {
        storage->P[i][j] = storage->P[j][i] = MIN_COVARIANCE;
      } else {
        storage->P[i][j] = storage->P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}

static void externalizeState(kalmanStorage_t* storage, state_t *state, uint32_t tick) {
  // position state is already in world frame
  state->position = (point_t){
    .timestamp = tick,
    .x = storage->S[STATE_X],
    .y = storage->S[STATE_Y],
    .z = storage->S[STATE_Z]
  };

  // velocity is in body frame and needs to be rotated to world frame
  state->velocity = (velocity_t){
    .timestamp = tick,
    .x = storage->R[0][0]*storage->S[STATE_PX] + storage->R[0][1]*storage->S[STATE_PY] + storage->R[0][2]*storage->S[STATE_PZ],
    .y = storage->R[1][0]*storage->S[STATE_PX] + storage->R[1][1]*storage->S[STATE_PY] + storage->R[1][2]*storage->S[STATE_PZ],
    .z = storage->R[2][0]*storage->S[STATE_PX] + storage->R[2][1]*storage->S[STATE_PY] + storage->R[2][2]*storage->S[STATE_PZ]
  };

  // convert the new attitude into Euler YPR
  float yaw = atan2f(2*(storage->q[1]*storage->q[2]+storage->q[0]*storage->q[3]) , powf(storage->q[0], 2) + powf(storage->q[1], 2) - powf(storage->q[2], 2) - powf(storage->q[3], 2));
  float pitch = asinf(-2*(storage->q[1]*storage->q[3] - storage->q[0]*storage->q[2]));
  float roll = atan2f(2*(storage->q[2]*storage->q[3]+storage->q[0]*storage->q[1]) , powf(storage->q[0], 2) - powf(storage->q[1], 2) - powf(storage->q[2], 2) + powf(storage->q[3], 2));

  // Save attitude, adjusted for the legacy CF2 body coordinate system
  state->attitude = (attitude_t){
    .timestamp = tick,
    .roll = roll*RAD_TO_DEG,
    .pitch = -pitch*RAD_TO_DEG,
    .yaw = yaw*RAD_TO_DEG
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  state->attitudeQuaternion = (quaternion_t){
    .timestamp = tick,
    .w = storage->q[0],
    .x = storage->q[1],
    .y = storage->q[2],
    .z = storage->q[3]
  };
}

static void update(kalmanStorage_t* storage, state_t *state, const uint32_t tick) {
  // If the client (via a parameter update) triggers an estimator reset:
  if (storage->resetEstimation) {
    init(storage);
    storage->resetEstimation = false;
  }

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool doneUpdate = false;
  uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
  // Decouple position states
  decoupleState(storage, STATE_X);
  decoupleState(storage, STATE_PX);
  decoupleState(storage, STATE_Y);
  decoupleState(storage, STATE_PY);
#endif

  /**
   * Add process noise every loop, rather than every prediction
   */
  // addProcessNoise(storage, (float)(osTick - storage->lastPNUpdate) / configTICK_RATE_HZ);
  // storage->lastPNUpdate = osTick;

  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */
  positionMeasurement_t position;
  while (hasPositionMeasurement(storage, &position))
  {
    updateWithPosition(storage, &position);
    doneUpdate = true;
  }

  distanceMeasurement_t dist;
  while (hasDistanceMeasurement(storage, &dist))
  {
    updateWithDistance(storage, &dist);
    doneUpdate = true;
  }

  /**
   * If an update has been made, the state is finalized:
   * - the attitude error is moved into the body attitude quaternion,
   * - the body attitude is converted into a rotation matrix for the next prediction, and
   * - correctness of the covariance matrix is ensured
   */

  if (doneUpdate) {
    finalize(storage, osTick);
    stateEstimatorAssertNotNaN();
  }

  /**
   * Finally, the internal state is externalized.
   * This is done every round, since the external state includes some sensor data
   */
  externalizeState(storage, state, osTick);
  stateEstimatorAssertNotNaN();
}

static bool enqueueMeasurement(xQueueHandle queue, void* measurement) {
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }
  return (result == pdTRUE);
}

static bool enqueueDistance(kalmanStorage_t* storage, distanceMeasurement_t *distance) {
  ASSERT(storage->isInit);
  return enqueueMeasurement(storage->distDataQueue, (void *)distance);
}

static bool enqueuePosition(kalmanStorage_t* storage, positionMeasurement_t *position) {
  ASSERT(storage->isInit);
  return enqueueMeasurement(storage->posDataQueue, (void *)position);
}

static point_t getPosition(kalmanStorage_t* storage) {
  point_t pos = {
    .x = storage->S[STATE_X],
    .y = storage->S[STATE_Y],
    .z = storage->S[STATE_Z]
  };

  return pos;
}

estimatorKalmanEngine_t estimatorKalmanEngine = {
  .init = init,
  .update = update,
  .enqueuePosition = enqueuePosition,
  .enqueueDistance = enqueueDistance,
  .getPosition = getPosition
};
