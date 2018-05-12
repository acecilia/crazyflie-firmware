#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

typedef struct {
  void (*init)(void);
} TwrSwarmAlgorithm;

extern TwrSwarmAlgorithm twrSwarmAlgorithm;

#endif /* TwrSwarmAlgorithm_h */
