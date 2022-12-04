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
#define NRF_DEBUG_MESSAGE 0
#define PRINT_PAYLOAD 0
#define PTX_BYTES_PER_SECOND 1
#define PRX_BYTES_PER_SECOND 1
#define COMMUNICATION_PERIOD 1
#define MEASUREMENT_PERIOD 1000
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
	  const uint32_t rxTimeout = 5;
	  uint8_t status;
	  uint8_t pipe;
	  uint8_t rxLng;
	  uint32_t startTick;
	  uint32_t byteCnt = 0;
#if  PTX_BYTES_PER_SECOND
  uint32_t tick = osKernelSysTick();
#endif
  /* Infinite loop */
  for(;;)
  {
	  NRF_configure(true);
	  while(1)
	  {
		  sprintf((char*)PTXsends, "%032ld", 0x7FFFFFFF - HAL_GetTick());
		  NRF_setW_TX_PAYLOAD(PTXsends, strlen((char*)PTXsends));

		  NRF_CEactivate();
		  startTick = osKernelSysTick();
		  while(startTick + rxTimeout > osKernelSysTick())
		  {
			  if(NRF_getInterrupt())
			  {
#if NRF_DEBUG_MESSAGE
				  printf("pTX IRQ\n");
#endif
				  startTick = 0;
				  break;
			  }
			  osDelay(1);
		  }

		  NRF_CEdeactivate();
		  status = NRF_getSTATUS();

		  if(status & (1 << MAX_RT))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pTX: MAX_RT\n");
#endif
			  /* For transmitter which haven't sent PAYLOAD - Flush and clear ISR*/
			  /* TODO: use and test NRF_setREUSE_TX_PL() */
			  NRF_setFLUSH_TX();
			  NRF_setSTATUS(1 << MAX_RT);
		  }

		  if(status & (1 << TX_DS))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pTX: TD_DS\n");
#endif
			  NRF_setSTATUS(1 << TX_DS);
		  }

		  if(status & (1 << RX_DR))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pTX: RX_DR\n");
#endif

			  pipe = ((status & RX_P_NO_2) |
					  (status & RX_P_NO_1) |
					  (status & RX_P_NO_0)) << 1;
			  if(RX_PIPE == pipe)
			  {
				  rxLng = NRF_getR_RX_PL_WID();
				  byteCnt += rxLng;
				  NRF_getR_RX_PAYLOAD(PTXreceives, rxLng);
#if PRINT_PAYLOAD
				  printf("pTX: (ACK) %d bytes = %s\n", width, (char*)PTXreceives);
#endif
				  NRF_setSTATUS(1 << RX_DR);
			  }
		  }

#if PTX_BYTES_PER_SECOND
		  if(tick + MEASUREMENT_PERIOD < osKernelSysTick())
		  {
			  printf("pTX: %ld B /%d ms\n", byteCnt, MEASUREMENT_PERIOD);
			  byteCnt = 0;
			  tick = osKernelSysTick();
		  }
#endif
		  osDelay(COMMUNICATION_PERIOD);
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
  const uint32_t rxTimeout = 100;
  uint8_t status;
  uint8_t pipe;
  uint8_t rxLng;
  uint32_t startTick;
  uint32_t byteCnt = 0;

#if  PRX_BYTES_PER_SECOND
  uint32_t tick = osKernelSysTick();
#endif
  /* Infinite loop */
  for(;;)
  {
	  NRF_configure_B(false);
	  while(1)
	  {

		  sprintf((char*)PRXsends, "%032ld", 0x7FFFFFFF - HAL_GetTick());
		  NRF_set_W_ACK_PAYLOAD_B(0, PRXsends, strlen((char*)PRXsends));

		  NRF_CEactivate_B();

		  startTick = osKernelSysTick();
		  while(startTick + rxTimeout > osKernelSysTick())
		  {
			  if(NRF_getInterrupt_B(true))
			  {
#if NRF_DEBUG_MESSAGE
				  printf("pRX IRQ\n");
#endif
				  startTick = 0;
				  break;
			  }
			  osDelay(1);
		  }

		  NRF_CEdeactivate_B();

		  status = NRF_getSTATUS_B();

		  if(startTick)
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pRX TO\n");
#endif
		  }

		  if(status & (1 << MAX_RT))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pRX: MAX_RT\n");
#endif
			  NRF_setFLUSH_TX_B();
			  NRF_setSTATUS_B(1 << MAX_RT);

		  }

		  if(status & (1 << RX_DR))
		  {

#if NRF_DEBUG_MESSAGE
			  printf("pRX: RX_DR\n");
#endif
			  pipe = ((status & RX_P_NO_2) |
					  (status & RX_P_NO_1) |
					  (status & RX_P_NO_0)) << 1;
			  if(RX_PIPE == pipe)
			  {
				  rxLng = NRF_getR_RX_PL_WID_B();
				  byteCnt += rxLng;
				  NRF_getR_RX_PAYLOAD_B(PRXreceives, rxLng);
#if PRINT_PAYLOAD
				  printf("pRX: %d bytes = %s\n", width, (char*)PRXreceives);
#endif
				  NRF_setSTATUS_B(1 << RX_DR);
			  }
		  }

		  if(status & (1 << TX_DS))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pRX: TX_DS\n");
#endif
			  NRF_setSTATUS_B(1 << TX_DS);
		  }

		  if(status & (1 << TX_FULL))
		  {
#if NRF_DEBUG_MESSAGE
			  printf("pRX: TX_FULL\n");
#endif
			  NRF_setSTATUS_B(1 << TX_FULL);
		  }

#if PRX_BYTES_PER_SECOND
		  if(tick + MEASUREMENT_PERIOD < osKernelSysTick())
		  {
			  printf("pRX: %ld B /%d ms\n", byteCnt, MEASUREMENT_PERIOD);
			  byteCnt = 0;
			  tick = osKernelSysTick();
		  }
#endif
		  /* For receiver which haven't sent ACK PAYLOAD - Flush */
		  /* TODO: use and test NRF_setREUSE_TX_PL() */
		  if((status & RX_P_NO_2) && (status & RX_P_NO_1) && (status & RX_P_NO_0))
		  {
			  NRF_setFLUSH_TX_B();
		  }

		  osDelay(COMMUNICATION_PERIOD);


	  }

  }
  /* USER CODE END startPrxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

