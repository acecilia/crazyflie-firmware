#include "leakyBucketEngine.h"

static void init(leakyBucketStorage_t* storage, unsigned int maximumCapacity) {
  storage->capacity = 0;
  storage->maximumCapacity = maximumCapacity;
}

static void fillBucket(leakyBucketStorage_t* storage) {
  if (storage->capacity < storage->maximumCapacity) {
    storage->capacity++;
  }
}

static void emptyBucket(leakyBucketStorage_t* storage) {
  if (storage->capacity > 0) {
    storage->capacity--;
  }
}

static bool isBucketEmpty(const leakyBucketStorage_t* storage) {
  return storage->capacity <= 0;
}

leakyBucketEngine_t leakyBucketEngine = {
  .init = init,
  .fillBucket = fillBucket,
  .emptyBucket = emptyBucket,
  .isBucketEmpty = isBucketEmpty
};
