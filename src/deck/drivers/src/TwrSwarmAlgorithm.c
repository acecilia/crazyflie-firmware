#include "TwrSwarmAlgorithm.h"

#include "dict.h"
#include "debug.h"

static dict *dct = NULL;

// TODO: This two functions for comparing and hashing uint8_t values may be taken to another file of the project, where makes more sense to place them

/**
 Comparator between two uint8_t values, to be used with the libdict library. See: https://github.com/fmela/libdict
 */
int uint8_cmp(const uint8_t *__s1, const uint8_t *__s2) {
  if (*__s1 == *__s2) {
    return 0;
  } else if (*__s1 < *__s2) {
    return -1;
  } else {
    return 1;
  }
}

/**
 Hash function of a uint8_t value, to be used with the libdict library. See: https://github.com/fmela/libdict
 */
unsigned uint8_hash(const uint8_t* k) {
  return *k;
}

static void init() {
  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new((dict_compare_func)uint8_cmp, (dict_hash_func)uint8_hash, 10);
}

static void test() {
  uint8_t key = 3;
  dict_insert_result result = dict_insert(dct, &key);

  if (result.inserted) {
    *result.datum_ptr = "example value";
    DEBUG_PRINT("inserted '%d': '%s'\n", key, (char *)*result.datum_ptr);
  } else {
    DEBUG_PRINT("Didnt work!");
  }

  uint8_t key2 = 3;
  void** search = dict_search(dct, &key2);
  if (search) {
    DEBUG_PRINT("found '%d': '%s'\n", key2, *(char **)search);
  } else {
    DEBUG_PRINT("'%d' not found!\n", key2);
  }
}

TwrSwarmAlgorithm twrSwarmAlgorithm = {
  .init = init,
  .test = test
};
