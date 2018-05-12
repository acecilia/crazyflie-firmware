#ifndef libdict_test_h
#define libdict_test_h

#include "libdict.h"

int benchmark_uint_keys_dict_insert(dictionary_t type);
int32_t benchmark_uint_keys_dict_insert_remove(dictionary_t type, int ms);
bool test_uint8_find(dictionary_t type);

#endif /* libdict_test_h */
