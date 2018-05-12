#include "TwrSwarmAlgorithm.h"

#include "libdict.h"
#include "debug.h"

static dict *dct = NULL;

static void init() {
  configure_dict_malloc();

  // Initialize the dictionary storing the rangings
  dct = hashtable2_dict_new(dict_uint8_cmp, dict_uint8_hash, 10);
}

TwrSwarmAlgorithm twrSwarmAlgorithm = {
  .init = init
};
