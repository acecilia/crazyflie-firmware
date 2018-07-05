/**
 Exposes the public API used for the leaky bucket algorithm in an Object Oriented way (and hides the functions that are only part of the internal implementation). See: https://en.wikipedia.org/wiki/Leaky_bucket
 */

#ifndef leakyBucketEngine_h
#define leakyBucketEngine_h

#include <stdbool.h>
#include "leakyBucketStorage.h"

typedef struct {
  void (*init)(leakyBucketStorage_t* storage, unsigned int maximumCapacity);
  void (*fillBucket)(leakyBucketStorage_t* storage);
  void (*emptyBucket)(leakyBucketStorage_t* storage);
  bool (*isBucketEmpty)(const leakyBucketStorage_t* storage);
} leakyBucketEngine_t;

extern leakyBucketEngine_t leakyBucketEngine;

#endif /* leakyBucketEngine_h */
