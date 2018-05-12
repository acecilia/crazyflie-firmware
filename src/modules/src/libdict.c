/**
 Helper functions, to be used with the libdict library. See: https://github.com/fmela/libdict
 */

#include "libdict.h"

/**
 Comparator between two uint8_t values
 */
int uint8_cmp(const uint8_t *__s1, const uint8_t *__s2) {
  if (*__s1 == *__s2) {
    return 0;
  } else if (*__s1 < *__s2) {
    return -1;
  } else {
    return 1;
  }
}

/**
 Hash function of a uint8_t value
 */
unsigned uint8_hash(const uint8_t* k) {
  return *k;
}

/**
 Comparator between two unsigned int values
 */
int uint_cmp(const unsigned *__s1, const unsigned *__s2) {
  if (*__s1 == *__s2) {
    return 0;
  } else if (*__s1 < *__s2) {
    return -1;
  } else {
    return 1;
  }
}

/**
 Hash function of an unsigned int value
 */
unsigned uint_hash(const unsigned* k) {
  return *k;
}

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
