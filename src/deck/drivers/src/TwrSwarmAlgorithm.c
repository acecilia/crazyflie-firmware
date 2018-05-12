#include "TwrSwarmAlgorithm.h"

#include "libdict.h"
#include "debug.h"

static dict *dct = NULL;

static void init() {
  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new((dict_compare_func)uint8_cmp, (dict_hash_func)uint8_hash, 10);
}

TwrSwarmAlgorithm twrSwarmAlgorithm = {
  .init = init
};
