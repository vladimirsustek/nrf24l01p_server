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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "nrf24l01p_driver.h"
#include "nrf24l01p_driver_B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMUNICATION_PERIOD 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t PRXsends[33] = {0};
uint8_t PTXsends[33] = {0};
uint8_t PRXreceives[33] = {0};
uint8_t PTXreceives[33] = {0};

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ptxTaskHandle;
osThreadId prxTaskHandle;
osMutexId myMutex01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void NRF_delay(uint32_t millisec) {osDelay(millisec);}
uint32_t NRF_appTimeOutStats(bool printStatistics, uint32_t* pByteCnt, uint32_t *pTick, uint32_t* pNoComPeriod);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void startPtxTask(void const * argument);
void startPrxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ptxTask */
  osThreadDef(ptxTask, startPtxTask, osPriorityNormal, 0, 128);
  ptxTaskHandle = osThreadCreate(osThread(ptxTask), NULL);

  /* definition and creation of prxTask */
  osThreadDef(prxTask, startPrxTask, osPriorityIdle, 0, 128);
  prxTaskHandle = osThreadCreate(osThread(prxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_startPtxTask */
/**
* @brief Function implementing the ptxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPtxTask */
void startPtxTask(void const * argument)
{
  /* USER CODE BEGIN startPtxTask */
	  const uint8_t RX_PIPE = 0;
	  const uint32_t TX_TIMEOUT = 5;
	  uint32_t noComPeriod = 0;
	  uint32_t byteCnt = 0;
	  uint32_t tick = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  if(0x0E != NRF_powerCycle(NRF_delay))
	  {
		  continue;
	  }

	  NRF_configure(true);

	  while(1)
	  {
		  sprintf((char*)PTXsends, "%032ld", 0x7FFFFFFF - HAL_GetTick());

		  NRF_setW_TX_PAYLOAD(PTXsends, strlen((char*)PTXsends));
		  NRF_activeRF(osKernelSysTick, NRF_delay, TX_TIMEOUT);
		  byteCnt += NRF_postProcess(RX_PIPE, PTXreceives);

		  if(0 == NRF_appTimeOutStats(false, &byteCnt, &tick, &noComPeriod)) { break;}

		  osDelay(10*COMMUNICATION_PERIOD);
	  }
  }
  /* USER CODE END startPtxTask */
}

/* USER CODE BEGIN Header_startPrxTask */
/**
* @brief Function implementing the prxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPrxTask */
void startPrxTask(void const * argument)
{
  /* USER CODE BEGIN startPrxTask */
  const uint8_t RX_PIPE = 0;
  const uint32_t RX_TIMEOUT = 100;
  uint32_t noComPeriod = 0;
  uint32_t byteCnt = 0;
  uint32_t tick = osKernelSysTick();

  /* Infinite loop */
  for(;;)
  {
	  if(0x0E != NRF_powerCycle_B(NRF_delay)) { continue; }

	  NRF_configure_B(false);

	  while(1)
	  {
		  sprintf((char*)PRXsends, "%032ld", 0x7FFFFFFF - HAL_GetTick());
		  NRF_set_W_ACK_PAYLOAD_B(0, PRXsends, strlen((char*)PRXsends));
		  NRF_activeRF_B(osKernelSysTick, NRF_delay, RX_TIMEOUT);
		  byteCnt += NRF_postProcess_B(RX_PIPE, PRXreceives);

		  if(0 == NRF_appTimeOutStats(true, &byteCnt, &tick, &noComPeriod)) { break;}

		  osDelay(COMMUNICATION_PERIOD);
	  }
  }
  /* USER CODE END startPrxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**
 * @brief Statistic function to check received bytes, compare timeout tick and invoke reset return value.
 * @param printStatistics when true prints statistic information.
 * @pByteCnt value passed by pointer.
 * @pTick value passed by pointer.
 * @pNoComPeriod value passed by pointer.
 *
 * @return 0 when reset needed, (uint32_t)(-1) otherwise.
 */
uint32_t NRF_appTimeOutStats(bool printStatistics, uint32_t* pByteCnt, uint32_t *pTick, uint32_t* pNoComPeriod)
{

const uint32_t SECOND_PERIOD = 1000;
const uint32_t FIVE_SECONDS_PERIOD = 5;

	uint32_t aux = (uint32_t)(-1);

	  if(*pTick + SECOND_PERIOD < osKernelSysTick())
	  {
		  if(printStatistics)
		  {
			  printf("pRX: %ld B /%ld ms\n", *pByteCnt, SECOND_PERIOD);
		  }

		  if(0 == *pByteCnt)
		  {
			  *pNoComPeriod = *pNoComPeriod + 1;
		  }
		  else
		  {
			  *pNoComPeriod = 0;
		  }

		  *pByteCnt = 0;
		  *pTick = osKernelSysTick();
	  }

	  if(*pNoComPeriod > FIVE_SECONDS_PERIOD)
	  {
		  *pNoComPeriod = 0;
		  aux = 0;
	  }

	  return aux;
}
/* USER CODE END Application */

