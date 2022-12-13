#include "fsm.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

#define TIMEEVENT_MAX_NUM 10

void vActiveConstructor(xActive * const me, vDispatchHandler dispatch) {
  me->dispatch = dispatch;
}

static void vActionEventLoop(void *pvParameters) {
  xActive *me = (xActive *) pvParameters;
  xEvent *e = (xEvent *) pvPortMalloc(sizeof(xEvent));
  
  static xEvent const xInitEvent = { INIT_SIG };
  (*me->dispatch)(me, &xInitEvent); /* initialize the Active Object */
#ifdef TEST
//  xReturn = xQueueSend(me->queue, (void *)&xInitEvent, 0U);
//  vActiveSend(me, &xInitEvent);
#endif
  
  while (1) {
    BaseType_t xReturn; /* return */
    /* wait for the event and receive it into object 'e' */
    xReturn = xQueueReceive(me->queue, e, portMAX_DELAY);
#ifdef DEBUG
    if (xReturn)
      printf("sig: %d\r\n", e->sig);
#endif
    configASSERT(xReturn);
    
    /* dispatch event to the active object 'me' */
    (*me->dispatch)(me, e);
  }
}



void vActiveStart(xActive * const me,
                  UBaseType_t prio,
//                  QueueHandle_t p_queue,
                  UBaseType_t queueLen,
                  configSTACK_DEPTH_TYPE stackDepth)
{
  UBaseType_t xReturn;
  static TaskHandle_t xEventLoop = NULL;
  configASSERT(me && (prio > tskIDLE_PRIORITY) && (prio < configMAX_PRIORITIES))
  
  me->queue = xQueueCreate(queueLen, sizeof(xEvent));
  configASSERT(me->queue);

  me->thread = prio;
  xReturn = xTaskCreate(
    vActionEventLoop,
    "eventloop",
    (uint16_t) stackDepth,
    (void *)me,
    prio,
    &xEventLoop);
  configASSERT(xReturn);
}

void vActiveSend(xActive * const me, xEvent const * const e) {
  BaseType_t xReturn;
  xReturn = xQueueSend(me->queue, e, 0U);
#ifdef DEBUG
  if (xReturn == pdTRUE) {
    printf("success.\r\n");
  }
#endif
  configASSERT(xReturn);
}

void vActiveSendFromISR(xActive * const me, xEvent const * const e) {
  BaseType_t xReturn;
  xReturn = xQueueSendFromISR(me->queue, e, 0U);
  configASSERT(xReturn);
}

static xTimeEvent *xTimeEvtList[TIMEEVENT_MAX_NUM]; /* all TimeEvents in the application */
static uint8_t uTimeEvtNum; /* current number of TimeEvents, default 0 */

void vTimeEventConstructor(xTimeEvent * const me, uSignal sig, xActive *act) {
  me->super.sig = sig;
  me->act = act;
  me->timeout = 0U;
  me->interval = 0U;

  taskENTER_CRITICAL();
  configASSERT(uTimeEvtNum < TIMEEVENT_MAX_NUM);
  xTimeEvtList[uTimeEvtNum] = me;
  ++uTimeEvtNum;
  taskEXIT_CRITICAL();
}

#ifdef TEST
void vTimeEventDelete(void) {
  taskENTER_CRITICAL();
  configASSERT(uTimeEvtNum > 0);
  --uTimeEvtNum;
  taskEXIT_CRITICAL();
}
#endif

void vTimeEventArm(xTimeEvent * const me, uint32_t timeout, uint32_t interval) {
  taskENTER_CRITICAL();
  me->timeout = timeout;
  me->interval = interval;
  taskEXIT_CRITICAL();
}

void vTimeEventTick(void) {
  uint8_t idx;
  for (idx = 0U; idx < uTimeEvtNum; ++idx) {
    xTimeEvent * const t = xTimeEvtList[idx];
    configASSERT(t); /* TimeEvent instance must be registered */
    if (t->timeout > 0U) {
      if (--(t->timeout) == 0U) { /* is it exiring now? */
        vActiveSend(t->act, &t->super);
        t->timeout = t->interval;
      }
    }
  }
}

void vTimeEventTickFromISR(void) {
  uint8_t idx;
  for (idx = 0U; idx < uTimeEvtNum; ++idx) {
    xTimeEvent * const t = xTimeEvtList[idx];
    configASSERT(t); /* TimeEvent instance must be registered */
    if (t->timeout > 0U) {
      if (--(t->timeout) == 0U) { /* is it exiring now? */
        vActiveSendFromISR(t->act, &t->super);
        t->timeout = t->interval;
      }
    }
  }
}
