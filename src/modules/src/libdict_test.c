#include "libdict_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

int32_t benchmark_uint_keys_dict_insert_remove(dictionary_t type, int ms) {
  uint32_t ticks_delta = M2T(ms);
  int32_t returnValue = 0;
  int32_t key = 0;

  dict *dct = create_dictionary(type, dict_uint_cmp, dict_uint_hash, 10);

  unsigned int start = xTaskGetTickCount();
  unsigned int end = start + ticks_delta;

  while (xTaskGetTickCount() < end) {
    // Because the crazyflie has limited memory, in order to perform the test we have to remove the inserted elements
    if (dict_insert(dct, &key).inserted && dict_remove(dct, &key).removed) {
      key++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  // dict_free(dct, key_val_free);
  return returnValue;
}

int benchmark_uint_keys_dict_insert(dictionary_t type) {
  int returnValue = 0;
  unsigned int key = 0;

  dict *dct = create_dictionary(type, dict_uint_cmp, dict_uint_hash, 10);

  while (xPortGetFreeHeapSize() > 10000) {
    if (dict_insert(dct, &key).inserted) {
      key++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  // dict_free(dct, key_val_free);
  return returnValue;
}

bool test_uint8_find(dictionary_t type) {
  bool returnValue = false;

  dict *dct = create_dictionary(type, dict_uint8_cmp, dict_uint8_hash, 10);

  uint8_t insertKey = 5;
  uint8_t searchKey = 5;
  char* value = "value example";

  dict_insert_result insert_result = dict_insert(dct, &insertKey);
  if (insert_result.inserted) {
    *insert_result.datum_ptr = value;

    void** search_result = dict_search(dct, &searchKey);
    if (search_result && strcmp(*(char **)search_result, value) == 0) {
      returnValue = true;
    }
  }

  // dict_clear(dct, key_val_free);
  return returnValue;
}
