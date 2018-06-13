#ifndef timersMocks_h
#define timersMocks_h

#include "FreeRTOS.h"
#include "timers.h"

void* pvTimerGetTimerID(const TimerHandle_t xTimer);
TimerHandle_t xTimerCreate( const char * const pcTimerName, const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void * const pvTimerID, TimerCallbackFunction_t pxCallbackFunction );
BaseType_t xTimerGenericCommand( TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t * const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait );

#endif /* timersMocks_h */
