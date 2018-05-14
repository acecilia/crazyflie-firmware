#ifndef libdict_test_tools_h
#define libdict_test_tools_h

#include "libdict.h"

int benchmark_uint_keys_dict_insert_speed(dictionary_t type, int ms);
int benchmark_uint_keys_dict_insert_memory(dictionary_t type);
bool test_uint8_insert_search(dictionary_t type);

#endif /* libdict_test_tools_h */
