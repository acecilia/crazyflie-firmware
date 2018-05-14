/**
 Helper functions, to be used with the libdict library. See: https://github.com/fmela/libdict
 */

#include "libdict.h"
#include "FreeRTOS.h"
#include "debug.h"

/**
 Make the library use the malloc and free functions from freeRTOS. Necessary to call this function once before using anything from the library
 */
void configure_dict_malloc() {
  dict_malloc_func = pvPortMalloc;
  dict_free_func = vPortFree;
}

/**
 Function to pass when calling dict_free
 */
void key_val_free(void *key, void *value) {
  // In order for this to work, you should have used dict_malloc_func before to allocate key and value
  dict_free_func(key);
  dict_free_func(value);
}

/**
 Comparator between two uint8_t values
 */
int dict_uint8_cmp(const void* k1, const void* k2) {
  const uint8_t a = *(const uint8_t*)k1;
  const uint8_t b = *(const uint8_t*)k2;
  return (a > b) - (a < b);
}

/**
 Hash function of a uint8_t value
 */
unsigned dict_uint8_hash(const void* k) {
  return *(uint8_t*)k;
}

/**
 Hash function of an unsigned int value
 */
unsigned dict_uint_hash(const void* k) {
  return *(unsigned int*)k;
}

/**
 Create a dictionary. Useful when you want to create different types of dictionaries in an automatic way (for example, using loops). For creating only one dictionary better use the function associated with it, instead of this one
 */
dict* create_dictionary(dictionary_t type, dict_compare_func cmp_func, dict_hash_func hash_func, unsigned hash_table_initial_size) {
  switch (type) {
    case height_balanced:
      return hb_dict_new(cmp_func);

    case path_reduction:
      return pr_dict_new(cmp_func);

    case red_black:
      return rb_dict_new(cmp_func);

    case treap:
      return tr_dict_new(cmp_func, NULL);

    case splay:
      return sp_dict_new(cmp_func);

    case skip_list:
      return skiplist_dict_new(cmp_func, 12);

    case weight_balanced:
      return wb_dict_new(cmp_func);

    case hashtable_separate_chaining:
      return hashtable_dict_new(cmp_func, hash_func, hash_table_initial_size);

    case hashtable_open_addressing:
      return hashtable2_dict_new(cmp_func, hash_func, hash_table_initial_size);
  }

  return NULL;
}
