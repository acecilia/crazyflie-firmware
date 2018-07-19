#ifndef estimatorKalmanEngine_h
#define estimatorKalmanEngine_h

#include <stdint.h>
#include "stabilizer_types.h"
#include "estimatorKalmanStorage.h"

/**
 A x,y,z vector with an associated standard deviation
 */
typedef struct {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float axis[3];
  };
  float stdDev;
} vec3Measurement_t;

typedef struct {
  void (*init)(estimatorKalmanStorage_t* storage, const point_t initialPosition, const velocity_t initialVelocity);
  void (*update)(estimatorKalmanStorage_t* storage);

  // Incorporation of additional data
  bool (*enqueueAcceleration)(const estimatorKalmanStorage_t* storage, const Axis3f* acceleration);
  bool (*enqueueAngularVelocity)(const estimatorKalmanStorage_t* storage, const Axis3f* angularVelocity);
  bool (*enqueuePosition)(const estimatorKalmanStorage_t* storage, const positionMeasurement_t* position);
  bool (*enqueueDistance)(const estimatorKalmanStorage_t* storage, const distanceMeasurement_t* distance);

  void (*getPosition)(const estimatorKalmanStorage_t* storage, point_t* position);
  void (*getState)(const estimatorKalmanStorage_t* storage, state_t* state);
} estimatorKalmanEngine_t;

extern estimatorKalmanEngine_t estimatorKalmanEngine;

#endif /* estimatorKalmanEngine_h */
