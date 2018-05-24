#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

uint32_t xTaskGetTickCount()
{
  return 1;
}

void *pvPortMalloc( size_t xWantedSize ) {
  return malloc(xWantedSize);
}

void vPortFree( void *pv ) {
  free(pv);
}
