#include <time.h>
#include "libdict_test.h"

int uint_keys_dict_insert_benchmark(dictionary_t type, int ms) {
  int returnValue = 0;
  unsigned key = 0;

  dict *dct = create_dictionary(type, (dict_compare_func)uint_cmp, (dict_hash_func)uint_hash, 10);

  time_t start = time(NULL);
  while (start < start + ms) {
    dict_insert_result result = dict_insert(dct, &key);

    if (result.inserted) {
      key++;
      returnValue++;
    } else {
      return -1;
    }
  }

  return returnValue;
}


bool uint8_test_find(dictionary_t type) {
  dict *dct = create_dictionary(type, (dict_compare_func)uint8_cmp, (dict_hash_func)uint8_hash, 10);

  uint8_t insertKey = 5;
  dict_insert_result result = dict_insert(dct, &insertKey);
  if (!result.inserted) {
    return false;
  }

  uint8_t searchKey = 5;
  void** search = dict_search(dct, &searchKey);
  if (!search) {
    return false;
  }

  return true;
}
