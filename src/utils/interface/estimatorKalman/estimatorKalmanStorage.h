#ifndef estimatorKalmanStorage_h
#define estimatorKalmanStorage_h

#include "FreeRTOS.h"
#include "queue.h"
#include "arm_math.h"

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t;

typedef struct {
  float S[STATE_DIM];

  // The queues to add data to the filter
  xQueueHandle posDataQueue;
  xQueueHandle distDataQueue;

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  float q[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // The covariance matrix
  float P[STATE_DIM][STATE_DIM];
  arm_matrix_instance_f32 Pm;

  // Internal variables. Note that static declaration results in default initialization (to 0)
  bool isInit;
  bool resetEstimation;
  /* No used */ // int32_t lastPrediction;
  /* No used */ // int32_t lastBaroUpdate;
  /* No used */ // int32_t lastPNUpdate;
  /* No used */ // Axis3f accAccumulator;
  /* No used */ // float thrustAccumulator;
  /* No used */ // Axis3f gyroAccumulator;
  /* No used */ // baro_t baroAccumulator;
  /* No used */ // uint32_t accAccumulatorCount;
  /* No used */ // uint32_t thrustAccumulatorCount;
  /* No used */ // uint32_t gyroAccumulatorCount;
  /* No used */ // uint32_t baroAccumulatorCount;
  /* No used */ // bool quadIsFlying;
  /* No used */ // int32_t lastTDOAUpdate;
  /* No used */ // float stateSkew;
  /* No used */ // float varSkew;
  /* No used */ // uint32_t lastFlightCmd;
  /* No used */ // uint32_t takeoffTime;
  /* No used */ // uint32_t tdoaCount;

  // Matrix to rotate the attitude covariances once updated
  float A[STATE_DIM][STATE_DIM];
  arm_matrix_instance_f32 Am;

  // The Kalman gain as a column vector
  float K[STATE_DIM];
  arm_matrix_instance_f32 Km;

  // Temporary matrices for the covariance updates
  float tmpNN1d[STATE_DIM * STATE_DIM];
  arm_matrix_instance_f32 tmpNN1m;

  float tmpNN2d[STATE_DIM * STATE_DIM];
  arm_matrix_instance_f32 tmpNN2m;

  float tmpNN3d[STATE_DIM * STATE_DIM];
  arm_matrix_instance_f32 tmpNN3m;

  float HTd[STATE_DIM * 1];
  arm_matrix_instance_f32 HTm;

  float PHTd[STATE_DIM * 1];
  arm_matrix_instance_f32 PHTm;
} kalmanStorage_t;

#endif /* estimatorKalmanStorage_h */
