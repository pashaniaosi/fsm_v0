/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
// main.h -> hal_library, PIN;
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include "fsm.h"
#include <stdbool.h>
#include "tim.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #define DEBUG          // debug info
// #define FSM_TEST       // test fsm model
// #define MULI_TASK_FSM  // test multi-task model
// #define LED_FSM        // LED FSM
#define DATA_MOD_FSM   // Data Collection Module FSM

#ifdef MULI_TASK_FSM
typedef enum {Sensor1, Sensor2, Sensor3} LAYER1EVENT;
#endif

#ifdef LED_FSM
enum xEventSignals {
  TIMEOUT_SIG = USER_SIG
};

typedef struct {
  xActive super;
  xTimeEvent te;
  bool is_led_on;
  uint32_t blink_time;
} xLedFsm;
#endif

#ifdef DATA_MOD_FSM
#define STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
enum xEventSignals {
  SENSOR1_SIG = USER_SIG,
  SENSOR2_SIG,
  SENSOR3_SIG,
};

typedef struct {
  xActive super;
  xTimeEvent te1;
  xTimeEvent te2;
  xTimeEvent te3;
  uint32_t interval1;
  uint32_t interval2;
  uint32_t interval3;
} xDataModFsm;

static TaskHandle_t xCPUTaskHandle;
StaticTask_t xCPUTCB;
StackType_t xCPUStack[STACK_SIZE];
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef LED_FSM
#define LED_AO_QUEUE_LEN 10U
#endif

#ifdef DATA_MOD_FSM
#define DATA_MOD_AO_QUEUE_LEN 10U
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#ifdef MULI_TASK_FSM
extern UART_HandleTypeDef huart1;
LAYER1STATE xLayer1States;

static TimerHandle_t xTimer1;
static TimerHandle_t xTimer2;
static TimerHandle_t xTimer3;
staitc TaskHandle_t xFSMTaskHandle;
static QueueHandle_t xLayer1Queue;
static int param = 1;
/* USER CODE END Variables */
/* Definitions for init */
osThreadId_t initHandle;
const osThreadAttr_t init_attributes = {
  .name = "init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void vTimer1Callback(TimerHandle_t xTimer1);
static void vTimer2Callback(TimerHandle_t xTimer2);
static void vTimer3Callback(TimerHandle_t xTimer3);

static void vFSMTaskEntry(void *parameter);
#endif

#ifdef LED_FSM
static void vLedFsmDispatch(xLedFsm * const me, xEvent const * const e) {
  switch (e->sig) {
    case INIT_SIG:
      LED_ON();
      me->is_led_on = true;      
      vTimeEventArm(&me->te, me->blink_time, me->blink_time);
#ifdef DEBUG
      printf("init\r\n");
#endif
      break;
    case TIMEOUT_SIG:
      printf("timeout\r\n");
      if (!me->is_led_on) { /* led is not on */
        LED_ON();
        me->is_led_on = true;
      } else {
        LED_OFF();
        me->is_led_on = false;
      }
      break;
  }
}

void vLedConstructor(xLedFsm * const me) {
  vActiveConstructor(&me->super, (vDispatchHandler)&vLedFsmDispatch);
  vTimeEventConstructor(&me->te, TIMEOUT_SIG, &me->super);
  me->is_led_on = false;
  me->blink_time = 1000;
}

static xLedFsm x_led_fsm;
xActive *act_led_fsm = &x_led_fsm.super;
#endif

#ifdef DATA_MOD_FSM
static void vDataModFsmDispatch(xDataModFsm * const me, xEvent const * const e) {
  switch (e->sig) {
    case INIT_SIG:
      LED_ON();
      vTimeEventArm(&me->te1, me->interval1, me->interval1);
      vTimeEventArm(&me->te2, me->interval2, me->interval2);
      vTimeEventArm(&me->te3, me->interval3, me->interval3);
      break;
    case SENSOR1_SIG:
      __NOP();
      printf("sensor1.\r\n");
      break;
    case SENSOR2_SIG:
      __NOP();
      printf("sensor2.\r\n");
      break;
    case SENSOR3_SIG:
      __NOP();
      printf("sensor3.\r\n");
    default:
      break;
  }
}

void vDataModConstructor(xDataModFsm * const me) {
  vActiveConstructor(&me->super, (vDispatchHandler)&vDataModFsmDispatch);
  vTimeEventConstructor(&me->te1, SENSOR1_SIG, &me->super);
  vTimeEventConstructor(&me->te2, SENSOR2_SIG, &me->super);
  vTimeEventConstructor(&me->te3, SENSOR3_SIG, &me->super);
  
  me->interval1 = 1000;
  me->interval2 = 2000;
  me->interval3 = 4000;
}

static xDataModFsm x_data_mod_fsm;
xActive *act_data_mod_fsm = &x_data_mod_fsm.super;

static void vCPUTaskEntry(void *parameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 8000;
  xLastWakeTime = xTaskGetTickCount();
  uint8_t *cpu_runinfo = (uint8_t *)pvPortMalloc(400 * sizeof(uint8_t));
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    memset(cpu_runinfo, 0, 400);
    vTaskList((char *) cpu_runinfo);
    printf("--------------------------------------------------\r\n");
    printf("Name                  Status  Prio   Stack     Seq\r\n");
    printf("%s", cpu_runinfo);
    printf("--------------------------------------------------\r\n");

    memset(cpu_runinfo, 0, 400);
    vTaskGetRunTimeStats((char *) cpu_runinfo);
    printf("Name                  Count          Usage\r\n");
    printf("%s", cpu_runinfo);
    printf("--------------------------------------------------\r\n");
  }
}
#endif
/* USER CODE END FunctionPrototypes */

void INIT(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  CPU_RunTime = 0UL;
}

__weak unsigned long getRunTimeCounterValue(void)
{
  return CPU_RunTime;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
  vTimeEventTickFromISR();
}
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
#ifdef LED_FSM
  vLedConstructor(&x_led_fsm);
  vActiveStart(
    act_led_fsm, tskIDLE_PRIORITY + 1, LED_AO_QUEUE_LEN, configMINIMAL_STACK_SIZE * 4
  );
#endif

#ifdef DATA_MOD_FSM
  vDataModConstructor(&x_data_mod_fsm);
  vActiveStart(
    act_data_mod_fsm, tskIDLE_PRIORITY + 1, DATA_MOD_AO_QUEUE_LEN, STACK_SIZE
  );
  xCPUTaskHandle = xTaskCreateStatic(vCPUTaskEntry, "cpuinfo", STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, xCPUStack, &xCPUTCB);
#endif

#ifdef FSM_TEST
  struct tmp_struct {
    QueueHandle_t p_queue;
  };
  
  struct tmp_struct * p_struct = (struct tmp_struct *) pvPortMalloc(sizeof(struct tmp_struct));
  uint16_t send_data = 5;
  uint16_t receive_data;
  BaseType_t xReturn;
  p_struct->p_queue = xQueueCreate(5, sizeof(uint16_t));
  xReturn = xQueueSend(p_struct->p_queue, &send_data, 0U);
  if (xReturn == pdTRUE) {
    printf("send success.\r\n");
  }
  xReturn = xQueueReceive(p_struct->p_queue, &receive_data, portMAX_DELAY);
  if (xReturn == pdTRUE) {
    printf("%d %d\r\n", send_data, receive_data);
  }
  while (1) {}
#endif

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
 #ifdef MULI_TASK_FSM
  xTimer1 = xTimerCreate("Timer1", 1, pdTRUE, (void *) 1, vTimer1Callback);
  if (xTimer1 != NULL) {
    xTimerStart(xTimer1, 0);
  }
  xTimer2 = xTimerCreate("Timer2", 2, pdTRUE, (void *) 2, vTimer2Callback);
  if (xTimer1 != NULL) {
    xTimerStart(xTimer2, 0);
  }
  xTimer3 = xTimerCreate("Timer3", 3, pdTRUE, (void *) 3, vTimer3Callback);
  if (xTimer1 != NULL) {
    xTimerStart(xTimer3, 0);
  }
  
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xLayer1Queue = xQueueCreate(5, sizeof(LAYER1EVENT));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of init */
  initHandle = osThreadNew(INIT, NULL, &init_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(vFSMTaskEntry, "fsm", (uint16_t) 128 * 4, (void *) NULL, tskIDLE_PRIORITY + 1, &xFSMTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
#endif
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_INIT */
#ifdef MULI_TASK_FSM
/**
  * @brief  Function implementing the init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INIT */
__weak void INIT(void *argument)
{
  /* USER CODE BEGIN INIT */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INIT */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void vTimer1Callback(TimerHandle_t xTimer1) {
  configASSERT(xTimer1);
  printf("%d\r\n", xTaskGetTickCount());
  LAYER1EVENT xEvent = Sensor1;
  BaseType_t xReturn = xQueueSend(xLayer1Queue, (void *) &xEvent, (TickType_t) 0);
  if (xReturn == errQUEUE_FULL)
    printf("queue faild\r\n");
}

static void vTimer2Callback(TimerHandle_t xTimer2) {
  configASSERT(xTimer2);
  printf("%d\r\n", xTaskGetTickCount());
  LAYER1EVENT xEvent = Sensor2;
  BaseType_t xReturn = xQueueSend(xLayer1Queue, (void *) &xEvent, (TickType_t) 0);
  if (xReturn == errQUEUE_FULL)
    printf("queue faild\r\n");
}

static void vTimer3Callback(TimerHandle_t xTimer3) {
  configASSERT(xTimer3);
  printf("%d\r\n", xTaskGetTickCount());
  LAYER1EVENT xEvent = Sensor3;
  BaseType_t xReturn = xQueueSend(xLayer1Queue, (void *) &xEvent, (TickType_t) 0);
  if (xReturn == errQUEUE_FULL)
    printf("queue faild\r\n");
}

static void vFSMTaskEntry(void *parameter) {
  LAYER1EVENT xEvent;
  BaseType_t xReturn;
  while (1) {
    xReturn = xQueueReceive(xLayer1Queue, &xEvent, portMAX_DELAY);
    if (xReturn == pdTRUE) {
      switch (xEvent) {
        case Sensor1:
          printf("sensor1\r\n");
          break;
        case Sensor2:
          printf("sensor2\r\n");
          break;
        case Sensor3:
          printf("sensor3\r\n");
          break;
        default:
          break;
      }
    }
    else
      printf("receive failed\r\n");
  }
}
#endif
/* USER CODE END Application */

