#include "TwrSwarmAlgorithm.h"

#include <stdlib.h>
#include "dict.h"
#include "debug.h"

static dict *dct = NULL;

int uint8cmp(const uint8_t *left, const uint8_t *right) {
  if (left == right) {
    return 0;
  } else if (left < right) {
    return -1;
  } else {
    return 1;
  }
}


static void init() {
  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new((dict_compare_func)uint8cmp, dict_str_hash, 10);
}

static void test() {
  uint8_t key = 1;
  dict_insert_result result = dict_insert(dct, &key);

  if (result.inserted) {
    *result.datum_ptr = "example value";
    DEBUG_PRINT("inserted '%d': '%s'\n", key, (char *)*result.datum_ptr);
  } else {
    DEBUG_PRINT("Didnt work!");
  }

  uint8_t key2 = 1;
  void** search = dict_search(dct, &key2);
  if (search) {
    DEBUG_PRINT("found '%d': '%s'\n", key2, *(char **)search);
  } else {
    DEBUG_PRINT("'%d' not found!\n", key2);
  }
}

TwrSwarmAlgorithm* twrSwarmAlgorithmInit(void) {
  TwrSwarmAlgorithm *twrSwarmAlgorithm = malloc(sizeof(TwrSwarmAlgorithm));
  twrSwarmAlgorithm->init = &init;
  twrSwarmAlgorithm->test = &test;
  return twrSwarmAlgorithm;
}
