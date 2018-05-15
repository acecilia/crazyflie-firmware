#ifndef libdict_test_tools_h
#define libdict_test_tools_h

#include "libdict.h"

bool dict_test_dynamic_uint8_insert_search(dictionary_t type);
int dict_benchmark_dynamic_uint_keys_insert_speed(dictionary_t type, int ms);
int dict_benchmark_dynamic_uint_keys_insert_memory(dictionary_t type);

bool dict_test_static_uint8_insert_search(dictionary_t type);
bool dict_test_static_uint8_insert_search_changing_key_value(dictionary_t type);
int dict_benchmark_static_uint_keys_insert_speed(dictionary_t type, int ms);

#endif /* libdict_test_tools_h */
