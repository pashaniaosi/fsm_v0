#ifndef FSM_H
#define FSM_H

#include "FreeRTOS.h"
#include "queue.h"

typedef uint16_t uSignal; /* event signal */

enum ReservedSignals {
  INIT_SIG, /* init event */
  USER_SIG  /* user defined event */
};

/* Event base class */
typedef struct {
  uSignal sig;
  /* other parameter in subclasses of Event */
} xEvent;

typedef struct Active xActive;

typedef void (*vDispatchHandler)(xActive * const me, xEvent const * const e);

/* Active Object base class */
struct Active {
  UBaseType_t thread; /* private thread */
  QueueHandle_t queue; /* message queue */
  
  vDispatchHandler dispatch; /* pointer ot the dispatch() function */
};

void vActiveConstructor(xActive * const me, vDispatchHandler dispatch);
void vActiveStart(xActive * const me,
                  UBaseType_t prio,
//                  QueueHandle_t p_queue,
                  UBaseType_t queueLen,
                  configSTACK_DEPTH_TYPE stackDepth);
void vActiveSend(xActive * const me, xEvent const * const e);
void vActiveSendFromISR(xActive * const me, xEvent const * const e);

/* Time Event class */
typedef struct {
  xEvent super;
  xActive *act;
  uint32_t timeout;  /* timeout counter; 0 means not armed */
  uint32_t interval; /* interval for periodic TimeEvent, 0 means one-shot */
} xTimeEvent;

void vTimeEventConstructor(xTimeEvent * const me, uSignal sig, xActive *act);
void vTimeEventArm(xTimeEvent * const me, uint32_t timeout, uint32_t interval);
void vTimeEventDisarm(xTimeEvent * const me);

void vTimeEventTick(void);
void vTimeEventTickFromISR(void);
#endif
