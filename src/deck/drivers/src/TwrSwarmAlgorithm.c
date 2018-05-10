#include "TwrSwarmAlgorithm.h"

#include "dict.h"
#include "debug.h"

static dict *dct = NULL;

int intcmp(const int *__s1, const int *__s2) {
  if (*__s1 == *__s2) {
    return 0;
  } else if (*__s1 < *__s2) {
    return -1;
  } else {
    return 1;
  }
}

static void init() {
  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new((dict_compare_func)intcmp, dict_str_hash, 10);
}

static void test() {
  int key = 3;
  dict_insert_result result = dict_insert(dct, &key);

  if (result.inserted) {
    *result.datum_ptr = "example value";
    DEBUG_PRINT("inserted '%d': '%s'\n", key, (char *)*result.datum_ptr);
  } else {
    DEBUG_PRINT("Didnt work!");
  }

  int key2 = 3;
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
