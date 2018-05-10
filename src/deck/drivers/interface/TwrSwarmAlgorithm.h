#ifndef TwrSwarmAlgorithm_h
#define TwrSwarmAlgorithm_h

typedef struct {
  void (*init)(void);
  void (*test)(void);
} TwrSwarmAlgorithm;

TwrSwarmAlgorithm* twrSwarmAlgorithmInit(void);

#endif /* TwrSwarmAlgorithm_h */
