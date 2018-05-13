#ifndef libdict_test_h
#define libdict_test_h

#include "libdict.h"

int benchmark_uint_keys_dict_insert_remove(dictionary_t type, int ms);
bool test_uint8_insert_search(dictionary_t type);

#endif /* libdict_test_h */
