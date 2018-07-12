#ifndef estimatorKalmanEngine_h
#define estimatorKalmanEngine_h

#include <stdint.h>
#include "stabilizer_types.h"
#include "estimatorKalmanStorage.h"

typedef struct {
  void (*init)(estimatorKalmanStorage_t* storage, const point_t initialPosition, const velocity_t initialVelocity);
  void (*update)(estimatorKalmanStorage_t* storage);

  // Incorporation of additional data
  bool (*enqueuePosition)(const estimatorKalmanStorage_t* storage, const positionMeasurement_t position);
  bool (*enqueueDistance)(const estimatorKalmanStorage_t* storage, const distanceMeasurement_t distance);

  void (*getPosition)(const estimatorKalmanStorage_t* storage, point_t* position);
  void (*getState)(const estimatorKalmanStorage_t* storage, state_t* state);
} estimatorKalmanEngine_t;

extern estimatorKalmanEngine_t estimatorKalmanEngine;

#endif /* estimatorKalmanEngine_h */
