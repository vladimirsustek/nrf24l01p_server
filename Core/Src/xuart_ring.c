#include <stdio.h>

#include "xuart_ring.h"
#include "string.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define UART_RX_BUFF_SIZE	(uint32_t)(128)

#define IS_LOWERCASE_HEX(x)	((x) >= 'a' && (x) <= 'f')
#define IS_UPPERCASE_HEX(x)	((x) >= 'A' && (x) <= 'F')
#define IS_NUMBER(x)        ((x) >= '0' && (x) <= '9')

static uint8_t inner_buffer[UART_RX_BUFF_SIZE] = {0};
static uint32_t inner_buff_head = 0;
static uint32_t inner_buff_tail = 0;
static uint32_t uart_lf_flag = 0;

static void enter_critical_region(void)
{
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}


static void exit_critical_region(void)
{
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}


int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}


void UARTstartISR(void)
{
	HAL_UART_Receive_IT(&huart3, inner_buffer, sizeof(uint8_t));
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		inner_buffer[inner_buff_head] = huart->Instance->RDR;
		inner_buff_head = (inner_buff_head + 1) % UART_RX_BUFF_SIZE;
		inner_buffer[inner_buff_head] = '\0';
		HAL_UART_Receive_IT(&huart3, inner_buffer + inner_buff_head, sizeof(uint8_t));
		uart_lf_flag = (huart->Instance->RDR == '\n') ? 1 : 0;
	}
}


uint8_t UARTisLFreceived(void)
{
    if(uart_lf_flag) {
        uart_lf_flag = 0;
        return 1;
    }
    return 0;
}


uint8_t UARTcopyBuffer(uint8_t * buffer, uint8_t lng)
{

    /* Error value */
    uint8_t length = 0xFF;

    if(buffer == NULL || lng <= UART_RX_BUFF_SIZE) {
        /* Return Error value when null pointer
           or buffer length is too small*/
        return length;
    }

    if (inner_buff_head > inner_buff_tail)
    {

        memcpy(buffer, (uint8_t*)inner_buffer + inner_buff_tail, inner_buff_head - inner_buff_tail);
        length = inner_buff_head - inner_buff_tail;
    }
    else
    {
        memcpy(buffer, (uint8_t*)inner_buffer + inner_buff_tail, UART_RX_BUFF_SIZE - inner_buff_tail);
        memcpy(buffer + (UART_RX_BUFF_SIZE - inner_buff_tail), (uint8_t*)inner_buffer, inner_buff_head);
        length = (UART_RX_BUFF_SIZE - inner_buff_tail) + inner_buff_head;
    }

    inner_buff_tail = inner_buff_head;

    return length;
}


uint8_t* UARTFetchReceivedLine(uint32_t* pLength)
{

    uint8_t * retP = NULL;

    /* +1 bigger for 0x0 appending - end of string for printf */
    static uint8_t outer_buffer[UART_RX_BUFF_SIZE + 1];
    memset(outer_buffer, '\0', UART_RX_BUFF_SIZE + 1);

    if(UARTisLFreceived())
    {
    	enter_critical_region();
        *pLength = UARTcopyBuffer(outer_buffer, UART_RX_BUFF_SIZE + 1);
        /* For formal printf string termination */
        outer_buffer[*pLength] = '\0';
        retP = outer_buffer;
        exit_critical_region();
    }

    return retP;
}


uint8_t* parseCommand(uint8_t* pBuff, uint32_t length)
{
	static uint8_t buffer[2];
	memset(buffer, '\0', 2);

	if (NULL == pBuff || length != strlen("X0123\n"))
	{
		printf("Incorrect input\n");
		return NULL;
	}

	for(uint32_t idx = 1; idx < 5; idx++)
	{
		if(!IS_LOWERCASE_HEX(pBuff[idx]) &&
		   !IS_UPPERCASE_HEX(pBuff[idx]) &&
		   !IS_NUMBER(pBuff[idx]))
		{
			printf("Incorrect input\n");
			return NULL;
		}
	}

	if(IS_LOWERCASE_HEX(pBuff[1]))
		buffer[0] = (pBuff[1] - 97) << 4;
	if(IS_UPPERCASE_HEX(pBuff[1]))
		buffer[0] = (pBuff[1] - 55) << 4;
	if(IS_NUMBER(pBuff[1]))
		buffer[0] = (pBuff[1] - 48) << 4;

	if(IS_LOWERCASE_HEX(pBuff[2]))
		buffer[0] |= (pBuff[2] - 97);
	if(IS_UPPERCASE_HEX(pBuff[2]))
		buffer[0] |= (pBuff[2] - 55);
	if(IS_NUMBER(pBuff[2]))
		buffer[0] |= (pBuff[2] - 48);

	if(IS_LOWERCASE_HEX(pBuff[3]))
		buffer[1] = (pBuff[3] - 97) << 4;
	if(IS_UPPERCASE_HEX(pBuff[3]))
		buffer[1] = (pBuff[3] - 55) << 4;
	if(IS_NUMBER(pBuff[3]))
		buffer[1] = (pBuff[3] - 48) << 4;

	if(IS_LOWERCASE_HEX(pBuff[4]))
		buffer[1] |= (pBuff[4] - 97);
	if(IS_UPPERCASE_HEX(pBuff[4]))
		buffer[1] |= (pBuff[4] - 55);
	if(IS_NUMBER(pBuff[4]))
		buffer[1] |= (pBuff[4] - 48);

	return buffer;
}


#if 0
pUI = UARTFetchReceivedLine(&length);
if(pUI)
{
	  pUI = parseCommand(pUI, length);

	  if (pUI != NULL)
	  {
		  uint8_t pRx[2] = {0};
		  HAL_GPIO_WritePin(NCS_1_GPIO_Port, NCS_1_Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive(&hspi1, pUI, pRx, 2, HAL_MAX_DELAY);
		  HAL_GPIO_WritePin(NCS_1_GPIO_Port, NCS_1_Pin, GPIO_PIN_SET);
		  printf("RX: 0x%02x%02x\n", pRx[0], pRx[1]);
		  printf("TX: 0x%02x%02x\n", pUI[0], pUI[1]);
	  }
}
#endif
