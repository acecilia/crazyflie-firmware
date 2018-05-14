#include "libdict_test_tools.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/**
 Insert a key from the dictionary continuously during the specified milliseconds, counting the number of times
 */
int benchmark_uint_keys_dict_insert(dictionary_t type, int ms) {
  int ticks_delta = M2T(ms);
  int returnValue = 0;
  unsigned int keyContent = 0;

  dict *dct = create_dictionary(type, dict_uint_cmp, dict_uint_hash, 10);

  unsigned int start = xTaskGetTickCount();
  unsigned int end = start + ticks_delta;

  while (xTaskGetTickCount() < end) {
    // In order to simulate a dynamic allocation of key and value, we use malloc
    unsigned int* key = pvPortMalloc(sizeof(unsigned int));
    *key = keyContent;

    bool inserted = dict_insert(dct, key).inserted;
    // Because the crazyflie has limited memory, in order to perform the test we have to clear the dictionary (otherwise the memory will be filled before finishing the benchmark)
    dict_clear(dct, key_val_free);

    if (inserted) {
      keyContent++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  dict_free(dct, key_val_free);
  return returnValue;
}

/**
 Insert a value in the dictionary and then search for it. Returns true if the value is found and its content is the same one as the inserted
 */
bool test_uint8_insert_search(dictionary_t type) {
  bool returnValue = false;

  dict *dct = create_dictionary(type, dict_uint8_cmp, dict_uint8_hash, 10);

  // In order to simulate a dynamic allocation of key and value, we use malloc
  uint8_t* insertKey = pvPortMalloc(sizeof(uint8_t));
  *insertKey = 5;
  uint8_t* searchKey = pvPortMalloc(sizeof(uint8_t));
  *searchKey = 5;
  char* valueContent = "Value example";
  char* value = pvPortMalloc(sizeof(char) * (strlen(valueContent) + 1));
  strcpy(value, valueContent);

  dict_insert_result insert_result = dict_insert(dct, insertKey);
  if (insert_result.inserted) {
    *insert_result.datum_ptr = value;

    void** search_result = dict_search(dct, searchKey);
    if (search_result && strcmp(*(char **)search_result, valueContent) == 0) {
      returnValue = true;
    }
  }

  dict_free(dct, key_val_free);
  return returnValue;
}
