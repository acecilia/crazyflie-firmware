#ifndef estimatorKalmanEngine_h
#define estimatorKalmanEngine_h

#include <stdint.h>
#include "stabilizer_types.h"
#include "estimatorKalmanStorage.h"

typedef struct {
  void (*init)(kalmanStorage_t* storage);
  void (*update)(kalmanStorage_t* storage, state_t *state, const uint32_t tick);

  // Incorporation of additional data
  bool (*enqueuePosition)(kalmanStorage_t* storage, positionMeasurement_t *position);
  bool (*enqueueDistance)(kalmanStorage_t* storage, distanceMeasurement_t *distance);

  point_t (*getPosition)(kalmanStorage_t* storage);
} estimatorKalmanEngine_t;

extern estimatorKalmanEngine_t estimatorKalmanEngine;

#endif /* estimatorKalmanEngine_h */
