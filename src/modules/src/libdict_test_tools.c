#include "libdict_test_tools.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/**
 Inserts keys dynamically allocated, continuously during the specified milliseconds, counting the number of times
 */
int dict_benchmark_dynamic_uint_keys_insert_speed(dictionary_t type, int ms) {
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
    dict_clear(dct, key_val_dynamic_free);

    if (inserted) {
      keyContent++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  dict_free(dct, key_val_dynamic_free);
  return returnValue;
}

/**
 Inserts keys dynamically allocated, continuously until the memory is filled, counting the number of times
 */
int dict_benchmark_dynamic_uint_keys_insert_memory(dictionary_t type) {
  int returnValue = 0;
  unsigned int keyContent = 0;

  dict *dct = create_dictionary(type, dict_uint_cmp, dict_uint_hash, 10);

  while (xPortGetFreeHeapSize() > 5000) {
    unsigned int* key = pvPortMalloc(sizeof(unsigned int));
    *key = keyContent;

    if (dict_insert(dct, key).inserted) {
      keyContent++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  dict_free(dct, key_val_dynamic_free);
  return returnValue;
}

/**
 Inserts a key and value dynamically allocated, and then search for it. Returns true if the value is found and its content is the same one as the inserted
 */
bool dict_test_dynamic_uint8_insert_search(dictionary_t type) {
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

  dict_free(dct, key_val_dynamic_free);
  return returnValue;
}

/**
 Inserts keys statically allocated, continuously during the specified milliseconds, counting the number of times
 */
int dict_benchmark_static_uint_keys_insert_speed(dictionary_t type, int ms) {
  int ticks_delta = M2T(ms);
  int returnValue = 0;
  unsigned int key = 0;

  dict *dct = create_dictionary(type, dict_uint_cmp, dict_uint_hash, 10);

  unsigned int start = xTaskGetTickCount();
  unsigned int end = start + ticks_delta;

  while (xTaskGetTickCount() < end) {
    bool inserted = dict_insert(dct, &key).inserted;
    // Because the crazyflie has limited memory, in order to perform the test we have to clear the dictionary (otherwise the memory will be filled before finishing the benchmark)
    dict_clear(dct, key_val_static_free);

    if (inserted) {
      key++;
      returnValue++;
    } else {
      returnValue = -1;
      break;
    }
  }

  dict_free(dct, key_val_static_free);
  return returnValue;
}

/**
 Inserts a key and value statically allocated, and then searches for it. Returns true if the value is found and its content is the same one as the inserted
 */
bool dict_test_static_uint8_insert_search(dictionary_t type) {
  bool returnValue = false;

  dict *dct = create_dictionary(type, dict_uint8_cmp, dict_uint8_hash, 10);

  uint8_t insertKey = 5;
  uint8_t searchKey = 5;
  char* value = "Value example";

  dict_insert_result insert_result = dict_insert(dct, &insertKey);
  if (insert_result.inserted) {
    *insert_result.datum_ptr = value;

    void** search_result = dict_search(dct, &searchKey);
    if (search_result && strcmp(*(char **)search_result, value) == 0) {
      returnValue = true;
    }
  }

  dict_free(dct, key_val_static_free);
  return returnValue;
}

/**
 Inserts a key and value statically allocated, and then searches for it. After, change the value of the key without changing the pointer, and search again with that pointer. Returns true if the key is found the first time, but not found the second time
 NOTE: this test may pass for some dictionary data structures, and not for others. Seems like the underlying implementation differs from one to another
 */
bool dict_test_static_uint8_insert_search_changing_key_value(dictionary_t type) {
  bool returnValue = false;

  dict *dct = create_dictionary(type, dict_uint8_cmp, dict_uint8_hash, 10);

  uint8_t key = 5;
  char* value = "Value example";

  dict_insert_result insert_result = dict_insert(dct, &key);
  if (insert_result.inserted) {
    *insert_result.datum_ptr = value;

    void** search_result = dict_search(dct, &key);
    if (search_result && strcmp(*(char **)search_result, value) == 0) {
      // Just a check to verify that the pointer does not change when changing the key value
      uint8_t* oldPointer = &key;
      key++;
      uint8_t* newPointer = &key;
      void **search_result2 = dict_search(dct, &key);
      if (oldPointer == newPointer && !search_result2) {
        returnValue = true;
      }
    }
  }

  dict_free(dct, key_val_static_free);
  return returnValue;
}
