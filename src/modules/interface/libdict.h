#ifndef libdict_h
#define libdict_h

#include "dict.h"

typedef enum {
  height_balanced,
  path_reduction,
  red_black,
  treap,
  splay,
  skip_list,
  weight_balanced,
  hashtable_separate_chaining,
  hashtable_open_addressing
} dictionary_t;

int uint8_cmp(const uint8_t *__s1, const uint8_t *__s2);
unsigned uint8_hash(const uint8_t* k);

int uint_cmp(const unsigned *__s1, const unsigned *__s2);
unsigned uint_hash(const unsigned* k);

dict* create_dictionary(dictionary_t type, dict_compare_func cmp_func, dict_hash_func hash_func, unsigned hash_table_initial_size);

#endif /* libdict_h */
