/**
 Exposes the storage required by the leaky bucket algorithm.
 */

#ifndef leakyBucketStorage_h
#define leakyBucketStorage_h

#include <stdint.h>

typedef struct {
  unsigned int capacity;
  unsigned int maximumCapacity;
} leakyBucketStorage_t;

#endif /* leakyBucketStorage_h */
