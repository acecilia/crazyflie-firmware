#ifndef estimatorKalmanEngine_h
#define estimatorKalmanEngine_h

#include <stdint.h>
#include "stabilizer_types.h"
#include "estimatorKalmanStorage.h"

typedef struct {
  void (*init)(estimatorKalmanStorage_t* storage);
  void (*update)(estimatorKalmanStorage_t* storage, state_t *state, const uint32_t tick);

  // Incorporation of additional data
  bool (*enqueuePosition)(estimatorKalmanStorage_t* storage, positionMeasurement_t *position);
  bool (*enqueueDistance)(estimatorKalmanStorage_t* storage, distanceMeasurement_t *distance);

  point_t (*getPosition)(estimatorKalmanStorage_t* storage);
} estimatorKalmanEngine_t;

extern estimatorKalmanEngine_t estimatorKalmanEngine;

#endif /* estimatorKalmanEngine_h */
