#include "libdict_test.h"
#include "FreeRTOS.h"
#include "debug.h"

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
 Printing tests
 */
void test_libdict() {
  int length = sizeof(type_array) / sizeof(dictionary_t);

  configure_dict_malloc();

  DEBUG_PRINT("### Starting libdict tests:\n");

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  DEBUG_PRINT("### Insert/search test:\n");
  for(int i = 0; i < length; i++) {
    char* r1 = test_uint8_insert_search(type_array[i]) ? "Ok" : "Failed";
    DEBUG_PRINT("%s [%s]\n", r1, getName(type_array[i]));
  }

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());

  int benchmark_duration_ms = 5000;
  DEBUG_PRINT("### Insert benchmark (%d ms):\n", benchmark_duration_ms);
  for(int i = 0; i < length; i++) {
    int r1 = benchmark_uint_keys_dict_insert(type_array[i], benchmark_duration_ms);
    DEBUG_PRINT("%d [%s]\n", r1, getName(type_array[i]));
  }

  DEBUG_PRINT("[Mem: %d bytes]\n", xPortGetFreeHeapSize());
}
