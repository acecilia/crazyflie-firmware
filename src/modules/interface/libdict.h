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

void configure_dict_malloc(void);

void key_val_dynamic_free(void *key, void *value);
void key_val_static_free(void *key, void *value);

int dict_uint8_cmp(const void *__s1, const void *__s2);
unsigned int dict_uint8_hash(const void* k);

int dict_uint64_cmp(const void *__s1, const void *__s2);
unsigned int dict_uint64_hash(const void* k);

unsigned int dict_uint_hash(const void* k);

dict* create_dictionary(dictionary_t type, dict_compare_func cmp_func, dict_hash_func hash_func, unsigned hash_table_initial_size);

#endif /* libdict_h */
