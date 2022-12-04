/*
 * xuart_ring.h
 *
 *  Created on: Nov 13, 2022
 *      Author: 42077
 */

#ifndef INC_XUART_RING_H_
#define INC_XUART_RING_H_

#include "stm32f7xx_hal.h"

#define BUFF_SIZE	(uint32_t)(128)

void UARTstartISR(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t UARTisLFreceived(void);
uint8_t UARTcopyBuffer(uint8_t * buffer, uint8_t lng);
uint8_t* UARTFetchReceivedLine(uint32_t* pLength);
uint8_t* parseCommand(uint8_t* pBuff, uint32_t length);

#endif /* INC_XUART_RING_H_ */
