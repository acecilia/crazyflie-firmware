#include "libdict_test.h"
#include "FreeRTOS.h"
#include "debug.h"

/**
 An array containing all the possible dictionary data structures, for iteration purposes
 */
static dictionary_t type_array[] = {
  height_balanced,
  path_reduction,
  red_black,
  treap,
  splay,
  skip_list,
  weight_balanced,
  hashtable_separate_chaining,
  hashtable_open_addressing
};

/**
 Return a printable name for the different dictionary data structures
 */
static char* getName(dictionary_t type) {
  switch (type) {
    case height_balanced: return "height_balanced";
    case path_reduction: return "path_reduction";
    case red_black: return "red_black";
    case treap: return "treap";
    case splay: return "splay";
    case skip_list: return "skip_list";
    case weight_balanced: return "weight_balanced";
    case hashtable_separate_chaining: return "hashtable_separate_chaining";
    case hashtable_open_addressing: return "hashtable_open_addressing";
    default: return "error";
  }
}

/**
 Printing tests. Requires disabling the watchdog.
 Notes: I have observed that running this in DEBUG mode leads to:
 * The crazyflie crashing
 * The results returned by the benchmak being approximately 1/4 as good as the results in release mode
 To disable the watchdog in release mode, look for calls to the function "watchdogInit" and comment them
 */
void test_libdict() {
  int length = sizeof(type_array) / sizeof(dictionary_t);

  configure_dict_malloc();

  DEBUG_PRINT("\n#### Starting libdict tests (REMEMBER: requires to disable the watchdog):\n");
  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  DEBUG_PRINT("\n### Insert/search test:\n");
  for(int i = 0; i < length; i++) {
    char* result = test_uint8_insert_search(type_array[i]) ? "Passed Ok" : "Failed";
    DEBUG_PRINT("%s [%s]\n", result, getName(type_array[i]));
  }
  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  DEBUG_PRINT("\n### Insert memory benchmark (higher is better):\n");
  for(int i = 0; i < length; i++) {
    int result = benchmark_uint_keys_dict_insert_memory(type_array[i]);
    DEBUG_PRINT("%d [%s] [Mem: %d bytes]\n", result, getName(type_array[i]), xPortGetFreeHeapSize());
  }
  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  int benchmark_duration_ms = 5000;
  DEBUG_PRINT("\n### Insert speed benchmark (%d ms, higher is better):\n", benchmark_duration_ms);
  for(int i = 0; i < length; i++) {
    int result = benchmark_uint_keys_dict_insert_speed(type_array[i], benchmark_duration_ms);
    DEBUG_PRINT("%d [%s]\n", result, getName(type_array[i]));
  }
  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  DEBUG_PRINT("\n#### Finished libdict tests\n\n");
}
