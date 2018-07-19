#include "estimatorKalmanEngine.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "param.h"

#include "math.h"
#include "arm_math.h"

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
#define MAX_POSITION (100) // Meters
#define MAX_VELOCITY (10)  // Meters per second

/*
// Initial variances, uncertain of position, but know we're stationary and roughly flat
// NOTE: only here as a reference. This values should not be used as static variables, but they should be passed to the filter as arguments in the init function
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;
*/

static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 1.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
static float measNoiseGyro_rollpitch = 0.1f; // radians per second
static float measNoiseGyro_yaw = 0.1f; // radians per second

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
  if(in == 0) {
    return 0;
  } else {
    float pOut = 0;
    arm_status result = arm_sqrt_f32(in, &pOut);
    configASSERT(ARM_MATH_SUCCESS == result);
    return pOut;
  }
}

/**
 * Other functions
 */

#define KALMAN_NAN_CHECK
#ifdef KALMAN_NAN_CHECK
static void stateEstimatorAssertNotNaN(estimatorKalmanStorage_t* storage) {
  for(int i = 0; i < STATE_DIM; i++) {
    if (isnan(storage->S[i])) {
      configASSERT(false);
    }
  }

  for(int i = 0; i < 3; i++) {
    if (isnan(storage->q[i])) {
      configASSERT(false);
    }
  }

  for(int i = 0; i < STATE_DIM; i++) {
    for(int j = 0; j < STATE_DIM; j++) {
      if (isnan(storage->P[i][j])) {
        configASSERT(false);
      }
    }
  }
}
#else
static void stateEstimatorAssertNotNaN(estimatorKalmanStorage_t* storage) {
  return;
}
#endif

// #define KALMAN_DECOUPLE_XY
#ifdef KALMAN_DECOUPLE_XY
// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(estimatorKalmanStorage_t* storage, stateIdx_t state) {
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

/******************************************
 * Temporary matrices used for calculations. Declared static for better usage of the memory: if not static, the stack may be too small for storing them, and stack overflow may occur
 *****************************************/

/* Used in the scalarUpdate function
 *****************************************/

// The Kalman gain as a column vector: the K parameter in the kalman filter update function
static float K[STATE_DIM];
static arm_matrix_instance_f32 Km = {STATE_DIM, 1, (float *)K};

// The H' parameter in the kalman filter update function (transposed)
static float Htd[STATE_DIM * 1];
static arm_matrix_instance_f32 Htm = {STATE_DIM, 1, Htd};

// The PH' parameter in the kalman filter update function (with H transposed)
static float PHtd[STATE_DIM * 1];
static arm_matrix_instance_f32 PHtm = {STATE_DIM, 1, PHtd};

/* Used in the finalize function
 *****************************************/

// Matrix to rotate the attitude covariances once updated
static float A[STATE_DIM][STATE_DIM];
static arm_matrix_instance_f32 Am = {STATE_DIM, STATE_DIM, (float *)A};

/* Temporary matrices for the covariance updates
 *****************************************/

static float tmpNN1d[STATE_DIM * STATE_DIM];
static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

static float tmpNN2d[STATE_DIM * STATE_DIM];
static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

static float tmpNN3d[STATE_DIM * STATE_DIM];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

/*****************************************/

/**
 * Queues to add data to the filter
 */

#define ACCELERATION_QUEUE_LENGTH (10)
#define ANGULAR_VELOCITY_QUEUE_LENGTH (10)
#define POSITION_QUEUE_LENGTH (10)
#define DISTANCE_QUEUE_LENGTH (10)

static bool enqueueData(xQueueHandle queue, const void* data) {
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, data, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, data, 0);
  }
  return (result == pdTRUE);
}

static bool enqueueAcceleration(const estimatorKalmanStorage_t* storage, const Axis3f* acceleration) {
  ASSERT(storage->isInit);
  return enqueueData(storage->positionDataQueue, (void*)acceleration);
}

static bool enqueueAngularVelocity(const estimatorKalmanStorage_t* storage, const Axis3f* angularVelocity) {
  ASSERT(storage->isInit);
  return enqueueData(storage->distanceDataQueue, (void*)angularVelocity);
}

static bool enqueuePosition(const estimatorKalmanStorage_t* storage, const positionMeasurement_t* position) {
  ASSERT(storage->isInit);
  return enqueueData(storage->positionDataQueue, (void*)position);
}

static bool enqueueDistance(const estimatorKalmanStorage_t* storage, const distanceMeasurement_t* distance) {
  ASSERT(storage->isInit);
  return enqueueData(storage->distanceDataQueue, (void*)distance);
}

static inline bool hasAccelerationData(estimatorKalmanStorage_t* storage, Axis3f* acceleration) {
  return (pdTRUE == xQueueReceive(storage->accelerationDataQueue, acceleration, 0));
}

static inline bool hasAngularVelocityData(estimatorKalmanStorage_t* storage, Axis3f* angularVelocity) {
  return (pdTRUE == xQueueReceive(storage->angularVelocityDataQueue, angularVelocity, 0));
}

static inline bool hasDistanceData(estimatorKalmanStorage_t* storage, distanceMeasurement_t* distance) {
  return (pdTRUE == xQueueReceive(storage->distanceDataQueue, distance, 0));
}

static inline bool hasPositionData(estimatorKalmanStorage_t* storage, positionMeasurement_t* position) {
  return (pdTRUE == xQueueReceive(storage->positionDataQueue, position, 0));
}

/**
 * Main Kalman Filter functions
 */

static void init(estimatorKalmanStorage_t* storage, const vec3Measurement_t* initialPosition, const vec3Measurement_t* initialVelocity, const vec3Measurement_t* initialAttitude) {
  // Reset the queues
  if (!storage->isInit) {
    storage->accelerationDataQueue = xQueueCreate(ACCELERATION_QUEUE_LENGTH, sizeof(Axis3f));
    storage->angularVelocityDataQueue = xQueueCreate(ANGULAR_VELOCITY_QUEUE_LENGTH, sizeof(Axis3f));
    storage->positionDataQueue = xQueueCreate(POSITION_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    storage->distanceDataQueue = xQueueCreate(DISTANCE_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
  } else {
    xQueueReset(storage->accelerationDataQueue);
    xQueueReset(storage->angularVelocityDataQueue);
    xQueueReset(storage->positionDataQueue);
    xQueueReset(storage->distanceDataQueue);
  }

  // Initialize the state
  // TODO: Can we initialize this more intelligently?
  storage->S[STATE_X] = initialPosition->value.x;
  storage->S[STATE_Y] = initialPosition->value.y;
  storage->S[STATE_Z] = initialPosition->value.z;
  storage->S[STATE_PX] = initialVelocity->value.z;
  storage->S[STATE_PY] = initialVelocity->value.y;
  storage->S[STATE_PZ] = initialVelocity->value.z;
  storage->S[STATE_D0] = initialAttitude->value.x;
  storage->S[STATE_D1] = initialAttitude->value.y;
  storage->S[STATE_D2] = initialAttitude->value.z;

  // Initialize the attitude quaternion
  for(int i=0; i<3; i++) {
    storage->q[i] = i == 0 ? 1 : 0;
  }

  // Set the initial rotation matrix to the identity. This only affects the first prediction step, since in the finalization, after shifting attitude errors into the attitude state, the rotation matrix is updated.
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      storage->R[i][j] = i==j ? 1 : 0;
    }
  }

  // Reset covariance storage
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      storage->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // Initialize state variances
  storage->P[STATE_X][STATE_X] = powf(initialPosition->stdDev.x, 2);
  storage->P[STATE_Y][STATE_Y] = powf(initialPosition->stdDev.y, 2);
  storage->P[STATE_Z][STATE_Z] = powf(initialPosition->stdDev.z, 2);

  storage->P[STATE_PX][STATE_PX] = powf(initialVelocity->stdDev.x, 2);
  storage->P[STATE_PY][STATE_PY] = powf(initialVelocity->stdDev.y, 2);
  storage->P[STATE_PZ][STATE_PZ] = powf(initialVelocity->stdDev.z, 2);

  storage->P[STATE_D0][STATE_D0] = powf(initialAttitude->stdDev.x, 2);
  storage->P[STATE_D1][STATE_D1] = powf(initialAttitude->stdDev.y, 2);
  storage->P[STATE_D2][STATE_D2] = powf(initialAttitude->stdDev.z, 2);

  // Initialize covariance matrix
  storage->Pm = (arm_matrix_instance_f32){ STATE_DIM, STATE_DIM, (float *)storage->P };

  storage->isInit = true;
}

static void addProcessNoise(estimatorKalmanStorage_t* storage, Axis3f *accStdDev, Axis3f *velStdDev, Axis3f *posStdDev, Axis3f *angularVelStdDev, Axis3f *angularPosStdDev, float dt) {
  if (dt > 0) {
    // Add process noise on position
    storage->P[STATE_X][STATE_X] += powf(accStdDev->x*dt*dt + velStdDev->x*dt + posStdDev->x, 2);
    storage->P[STATE_Y][STATE_Y] += powf(accStdDev->y*dt*dt + velStdDev->y*dt + posStdDev->y, 2);
    storage->P[STATE_Z][STATE_Z] += powf(accStdDev->z*dt*dt + velStdDev->z*dt + posStdDev->z, 2);

    // Add process noise on velocity
    storage->P[STATE_PX][STATE_PX] += powf(accStdDev->x*dt + velStdDev->x, 2);
    storage->P[STATE_PY][STATE_PY] += powf(accStdDev->y*dt + velStdDev->y, 2);
    storage->P[STATE_PZ][STATE_PZ] += powf(accStdDev->z*dt + velStdDev->z, 2);

    // Add process noise on attitude
    storage->P[STATE_D0][STATE_D0] += powf(angularVelStdDev->x*dt + angularPosStdDev->x, 2);
    storage->P[STATE_D1][STATE_D1] += powf(angularVelStdDev->y*dt + angularPosStdDev->y, 2);
    storage->P[STATE_D2][STATE_D2] += powf(angularVelStdDev->z*dt + angularPosStdDev->z, 2);
  }

  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*storage->P[i][j] + 0.5f*storage->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        storage->P[i][j] = storage->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        storage->P[i][j] = storage->P[j][i] = MIN_COVARIANCE;
      } else {
        storage->P[i][j] = storage->P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN(storage);
}

static void predict(estimatorKalmanStorage_t* storage, Axis3f* acc, Axis3f* gyro, float dt) {
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d, skew are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[STATE_X][STATE_X] = 1;
  A[STATE_Y][STATE_Y] = 1;
  A[STATE_Z][STATE_Z] = 1;

  A[STATE_PX][STATE_PX] = 1;
  A[STATE_PY][STATE_PY] = 1;
  A[STATE_PZ][STATE_PZ] = 1;

  A[STATE_D0][STATE_D0] = 1;
  A[STATE_D1][STATE_D1] = 1;
  A[STATE_D2][STATE_D2] = 1;

  // position from body-frame velocity
  A[STATE_X][STATE_PX] = storage->R[0][0]*dt;
  A[STATE_Y][STATE_PX] = storage->R[1][0]*dt;
  A[STATE_Z][STATE_PX] = storage->R[2][0]*dt;

  A[STATE_X][STATE_PY] = storage->R[0][1]*dt;
  A[STATE_Y][STATE_PY] = storage->R[1][1]*dt;
  A[STATE_Z][STATE_PY] = storage->R[2][1]*dt;

  A[STATE_X][STATE_PZ] = storage->R[0][2]*dt;
  A[STATE_Y][STATE_PZ] = storage->R[1][2]*dt;
  A[STATE_Z][STATE_PZ] = storage->R[2][2]*dt;

  // position from attitude error
  A[STATE_X][STATE_D0] = (storage->S[STATE_PY]*storage->R[0][2] - storage->S[STATE_PZ]*storage->R[0][1])*dt;
  A[STATE_Y][STATE_D0] = (storage->S[STATE_PY]*storage->R[1][2] - storage->S[STATE_PZ]*storage->R[1][1])*dt;
  A[STATE_Z][STATE_D0] = (storage->S[STATE_PY]*storage->R[2][2] - storage->S[STATE_PZ]*storage->R[2][1])*dt;

  A[STATE_X][STATE_D1] = (- storage->S[STATE_PX]*storage->R[0][2] + storage->S[STATE_PZ]*storage->R[0][0])*dt;
  A[STATE_Y][STATE_D1] = (- storage->S[STATE_PX]*storage->R[1][2] + storage->S[STATE_PZ]*storage->R[1][0])*dt;
  A[STATE_Z][STATE_D1] = (- storage->S[STATE_PX]*storage->R[2][2] + storage->S[STATE_PZ]*storage->R[2][0])*dt;

  A[STATE_X][STATE_D2] = (storage->S[STATE_PX]*storage->R[0][1] - storage->S[STATE_PY]*storage->R[0][0])*dt;
  A[STATE_Y][STATE_D2] = (storage->S[STATE_PX]*storage->R[1][1] - storage->S[STATE_PY]*storage->R[1][0])*dt;
  A[STATE_Z][STATE_D2] = (storage->S[STATE_PX]*storage->R[2][1] - storage->S[STATE_PY]*storage->R[2][0])*dt;

  // body-frame velocity from body-frame velocity
  A[STATE_PX][STATE_PX] = 1; // Drag negligible
  A[STATE_PY][STATE_PX] =-gyro->z*dt;
  A[STATE_PZ][STATE_PX] = gyro->y*dt;

  A[STATE_PX][STATE_PY] = gyro->z*dt;
  A[STATE_PY][STATE_PY] = 1; // Drag negligible
  A[STATE_PZ][STATE_PY] =-gyro->x*dt;

  A[STATE_PX][STATE_PZ] =-gyro->y*dt;
  A[STATE_PY][STATE_PZ] = gyro->x*dt;
  A[STATE_PZ][STATE_PZ] = 1; // Drag negligible

  // body-frame velocity from attitude error
  A[STATE_PX][STATE_D0] = 0;
  A[STATE_PY][STATE_D0] = storage->R[2][2]*dt;
  A[STATE_PZ][STATE_D0] = storage->R[2][1]*dt;

  A[STATE_PX][STATE_D1] = storage->R[2][2]*dt;
  A[STATE_PY][STATE_D1] = 0;
  A[STATE_PZ][STATE_D1] = storage->R[2][0]*dt;

  A[STATE_PX][STATE_D2] = storage->R[2][1]*dt;
  A[STATE_PY][STATE_D2] = storage->R[2][0]*dt;
  A[STATE_PZ][STATE_D2] = 0;

  // attitude error from attitude error
  /**
   * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
   * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
   * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
   * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
   *
   * This comes from a second order approximation to:
   * Sigma_post = exps(-d) Sigma_pre exps(-d)'
   *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
   * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
   *
   * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
   * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
   */
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;

  A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
  A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

  A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
  A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

  A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
  A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
  A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &storage->Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &storage->Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;

  // Position updates in the body frame (will be rotated to inertial frame)
  dx = storage->S[STATE_PX] * dt + acc->x * dt2 / 2.0f;
  dy = storage->S[STATE_PY] * dt + acc->y * dt2 / 2.0f;
  dz = storage->S[STATE_PZ] * dt + acc->z * dt2 / 2.0f;

  // Position update
  storage->S[STATE_X] += storage->R[0][0] * dx + storage->R[0][1] * dy + storage->R[0][2] * dz;
  storage->S[STATE_Y] += storage->R[1][0] * dx + storage->R[1][1] * dy + storage->R[1][2] * dz;
  storage->S[STATE_Z] += storage->R[2][0] * dx + storage->R[2][1] * dy + storage->R[2][2] * dz;

  // Keep previous time step's state for the update
  tmpSPX = storage->S[STATE_PX];
  tmpSPY = storage->S[STATE_PY];
  tmpSPZ = storage->S[STATE_PZ];

  // Body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
  storage->S[STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - storage->R[2][0]);
  storage->S[STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - storage->R[2][1]);
  storage->S[STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - storage->R[2][2]);

  if(storage->S[STATE_Z] < 0) {
    storage->S[STATE_Z] = 0;
    storage->S[STATE_PX] = 0;
    storage->S[STATE_PY] = 0;
    storage->S[STATE_PZ] = 0;
  }

  // Attitude update (rotate by gyroscope), we do this in quaternions this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // Compute the quaternion values in [w,x,y,z] order
  if(dtwx != 0 && dtwy != 0 && dtwz != 0) {
    float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
    float ca = arm_cos_f32(angle/2.0f);
    float sa = arm_sin_f32(angle/2.0f);
    float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

    // Rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = (dq[0]*storage->q[0] - dq[1]*storage->q[1] - dq[2]*storage->q[2] - dq[3]*storage->q[3]);
    float tmpq1 = (1.0f)*(dq[1]*storage->q[0] + dq[0]*storage->q[1] + dq[3]*storage->q[2] - dq[2]*storage->q[3]);
    float tmpq2 = (1.0f)*(dq[2]*storage->q[0] - dq[3]*storage->q[1] + dq[0]*storage->q[2] + dq[1]*storage->q[3]);
    float tmpq3 = (1.0f)*(dq[3]*storage->q[0] + dq[2]*storage->q[1] - dq[1]*storage->q[2] + dq[0]*storage->q[3]);

    // Normalize and store the result
    float norm = arm_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
    storage->q[0] = tmpq0/norm;
    storage->q[1] = tmpq1/norm;
    storage->q[2] = tmpq2/norm;
    storage->q[3] = tmpq3/norm;
  } else {
    storage->q[0] = 0;
    storage->q[1] = 0;
    storage->q[2] = 0;
    storage->q[3] = 0;
  }

  stateEstimatorAssertNotNaN(storage);
}

static void scalarUpdate(estimatorKalmanStorage_t* storage, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise) {
  configASSERT(Hm->numRows == 1);
  configASSERT(Hm->numCols == STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &Htm);
  mat_mult(&storage->Pm, &Htm, &PHtm); // PH'
  float HPHt = 0; // HPH'
  for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHt += Hm->pData[i]*PHtd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  configASSERT(!isnan(HPHt));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  float R = powf(stdMeasNoise, 2);
  for (int i=0; i<STATE_DIM; i++) {
    K[i] = PHtd[i]/(HPHt + R); // kalman gain = (PH' (HPH' + R )^-1)
    storage->S[i] = storage->S[i] + K[i] * error; // state update
  }
  stateEstimatorAssertNotNaN(storage);

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<STATE_DIM; i++) {
    tmpNN1d[STATE_DIM*i+i] -= 1;
  } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &storage->Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &storage->Pm); // (KH - I)*P*(KH - I)'
  stateEstimatorAssertNotNaN(storage);

  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float v = K[i] * R * K[j];
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

  stateEstimatorAssertNotNaN(storage);
}

static void updateWithPosition(estimatorKalmanStorage_t* storage, positionMeasurement_t* xyz) {
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

static void updateWithDistance(estimatorKalmanStorage_t* storage, distanceMeasurement_t* d) {
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

static void finalize(estimatorKalmanStorage_t* storage) {
  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = storage->S[STATE_D0];
  float v1 = storage->S[STATE_D1];
  float v2 = storage->S[STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10)) {
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

    A[STATE_X][STATE_X] = 1;
    A[STATE_Y][STATE_Y] = 1;
    A[STATE_Z][STATE_Z] = 1;

    A[STATE_PX][STATE_PX] = 1;
    A[STATE_PY][STATE_PY] = 1;
    A[STATE_PZ][STATE_PZ] = 1;

    A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
    A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

    A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
    A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

    A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
    A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
    A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &storage->Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &storage->Pm); //APA'
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

  stateEstimatorAssertNotNaN(storage);
}

static void update(estimatorKalmanStorage_t* storage) {
  uint32_t tick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
  // Decouple position states
  decoupleState(storage, STATE_X);
  decoupleState(storage, STATE_PX);
  decoupleState(storage, STATE_Y);
  decoupleState(storage, STATE_PY);
#endif

  /*
   * Prediction step + noise addition
   */

  float dt = (float)((tick - storage->lastUpdate) / configTICK_RATE_HZ);

  Axis3f acceleration = {.x = 0, .y = 0, .z = 0 };
  bool accelerationDataReceived = false;
  while (hasAccelerationData(storage, &acceleration)) {
    accelerationDataReceived = true;
    // Accumulate acceleration. Not done because was not needed yet
  }

  Axis3f angularVelocity = {.x = 0, .y = 0, .z = 0 };
  bool angularVelocityDataReceived = false;
  while (hasAngularVelocityData(storage, &angularVelocity)) {
    angularVelocityDataReceived = true;
    // Accumulate angular velocity. Not done because was not needed yet
  }

  predict(storage, &acceleration, &angularVelocity, dt);

  // Add process noise every loop, rather than every prediction
  Axis3f accStdDev = accelerationDataReceived ? (Axis3f){.x = procNoiseAcc_xy, .y = procNoiseAcc_xy, .z = procNoiseAcc_z } : (Axis3f){.x = 0, .y = 0, .z = 0 };
  Axis3f angularVelStdDev = angularVelocityDataReceived == true ? (Axis3f){.x = measNoiseGyro_rollpitch, .y = measNoiseGyro_rollpitch, .z = measNoiseGyro_yaw } : (Axis3f){.x = 0, .y = 0, .z = 0 };
  Axis3f velStdDev = {.x = procNoiseVel, .y = procNoiseVel, .z = procNoiseVel };
  Axis3f posStdDev = {.x = procNoisePos, .y = procNoisePos, .z = procNoisePos };
  Axis3f angularPosStdDev = {.x = procNoiseAtt, .y = procNoiseAtt, .z = procNoiseAtt };
  addProcessNoise(storage, &accStdDev, &velStdDev, &posStdDev, &angularVelStdDev, &angularPosStdDev, dt);

  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  positionMeasurement_t position;
  while (hasPositionData(storage, &position)) {
    updateWithPosition(storage, &position);
  }

  distanceMeasurement_t dist;
  while (hasDistanceData(storage, &dist)) {
    updateWithDistance(storage, &dist);
  }

  /**
   * The state is finalized:
   * - the attitude error is moved into the body attitude quaternion,
   * - the body attitude is converted into a rotation matrix for the next prediction, and
   * - correctness of the covariance matrix is ensured
   */

  finalize(storage);

  // Save state to the storage
  storage->lastUpdate = tick;

  stateEstimatorAssertNotNaN(storage);
}

/*
 * Getters
 */

static void getPosition(const estimatorKalmanStorage_t* storage, point_t* position) {
  ASSERT(storage->isInit);

  position->x = storage->S[STATE_X];
  position->y = storage->S[STATE_Y];
  position->z = storage->S[STATE_Z];
}

static void getState(const estimatorKalmanStorage_t* storage, state_t *state) {
  ASSERT(storage->isInit);

  uint32_t tick = xTaskGetTickCount();

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

estimatorKalmanEngine_t estimatorKalmanEngine = {
  .init = init,
  .update = update,
  .enqueueAcceleration = enqueueAcceleration,
  .enqueueAngularVelocity = enqueueAngularVelocity,
  .enqueuePosition = enqueuePosition,
  .enqueueDistance = enqueueDistance,
  .getPosition = getPosition,
  .getState = getState
};
