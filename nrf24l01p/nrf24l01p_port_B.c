/*
 * nrf24l01p_port_B.c
 *
 *  Created on: Nov 19, 2022
 *      Author: Vladimir Sustek, MSc
 */


/**
  * @brief  Porting layer (SPI, CE GPIO and ISR GPIO) for NRF24L01+.
  *
  * @note This driver is dedicated for second (B) NRF24L01+ within one host MCU.
  *
  * NRF24L01+ needs following peripheral:
  *
  * SPI to operate with pins (max SCK 10MHz):
  * - SCK
  * - MISO
  * - MOSI
  * - CHIP SELECT (May be a SW-controlled GPIO pin)
  *
  * GPIO to control Chip Enable (NRF24L01+ RF Active pin):
  * - a Chip Enable GPIO pin in push-pull mode 0-3V3
  *
  * IRQ pin to detect special cases of operation (received/transmitted):
  * - an IRQ pin with detection on falling edge (3V3 to 0V)
  *
  * For different platforms needs to be this layer implemented accordingly:
  *
  * 1) nrfport_spiTransaction_B(): correct Chip Select functions and SPI transaction
  * needed. This function is universal and may send/receive number of bytes (lng).
  *
  * 2) nrfport_ceHigh_B(), nrfport_ceLow_B(): correct GPIO function (or setting a
  * correct bit in GPIO register) must be assigned to fulfill  Chip Enable NRF's
  * functionality (Chip Enable represents RF activation - send or receive).
  *
  * 3) nrfport_enISR_B(), nrfport_disISR_B(): Correct ISR function (or setting a
  * correct bit in ISR register) in order to disable the NRF's interrupts.
  */

#include "nrf24l01p_defines.h"

/* Platform dependent includes - differ across different platforms */
#include "spi.h"
#include "gpio.h"


/**
  * @brief Initialization of SPI + Chip select , CE GPIOs and IRQ.
  *
  * @note This function must initialize all peripherals for SPI, CE
  * and IRQ functionality. However initialization may be handled
  * externally and thus this function may be keep mocked returning 0.
  *
  * @return 0 when initialization passed.
  */
uint8_t nrfport_initHosthw_B()
{
	uint8_t aux = 0;
	return aux;
}


/**
  * @brief Universal SPI write and read function including Chip Select handling.
  *
  * @param tx pointer to buffer for transmit
  * @param tx pointer to buffer for receive
  * @param tx length of data to be transmitted
  *
  * @note NRF24L01+ is sampling input data on rising edge with LOW clock in idle:
  * This SPI may be described as: SPI_POLARITY_LOW and SPI_PHASE_1EDGE.
  * During the transaction must be Chip select low from 1st to last byte of lng.
  */
void nrfport_spiTransaction_B(uint8_t* tx, uint8_t *rx, uint32_t lng)
{
	if(NULL == tx || NULL == rx) return;

	HAL_GPIO_WritePin(NCS_R_GPIO_Port, NCS_R_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, lng, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NCS_R_GPIO_Port, NCS_R_Pin, GPIO_PIN_SET);
}


/**
  * @brief Universal SPI write a byte function.
  *
  * @param adr address of NRF24L01+ register
  * @param data content which will be in the register written
  *
  */
void nrfport_writeByte_B(uint8_t adr, uint8_t data)
{
	uint8_t rx[WR_BYTE_LNG] = {0};
	uint8_t tx[WR_BYTE_LNG] = {0};

	tx[WR_BYTE_CMD] = CMD_W_REGISTER | adr;
	tx[WR_BYTE_PLD] = data;

	nrfport_spiTransaction_B(tx, rx, WR_BYTE_LNG);
}


/**
  * @brief Universal SPI read a byte function.
  *
  * @param adr address of NRF24L01+ register
  *
  * @return data content of the register
  */
uint8_t nrfport_readByte_B(uint8_t adr)
{
	uint8_t rx[RD_BYTE_LNG] = {0};
	uint8_t tx[RD_BYTE_LNG] = {0};

	tx[RD_BYTE_CMD] = CMD_R_REGISTER | adr;

	nrfport_spiTransaction_B(tx, rx, WR_BYTE_LNG);

	return rx[RD_BYTE_PLD];
}

/**
  * @brief Set RF Active pin high.
  */
void nrfport_ceHigh_B(void)
{
	HAL_GPIO_WritePin(CE_R_GPIO_Port, CE_R_Pin, GPIO_PIN_SET);
}


/**
  * @brief Set RF Active pin low.
  */
void nrfport_ceLow_B(void)
{
	HAL_GPIO_WritePin(CE_R_GPIO_Port, CE_R_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Power up device's supply line (depends on implementation N-FET, P-FET NPN, PNP)
 */
void nrfport_powerUp_B(void)
{
	/* P-FET implementation */
	HAL_GPIO_WritePin(PWR_R_GPIO_Port, PWR_R_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Power down device's supply line (depends on implementation N-FET, P-FET NPN, PNP)
 */
void nrfport_powerDown_B(void)
{
	/* P-FET implementation */
	HAL_GPIO_WritePin(PWR_R_GPIO_Port, PWR_R_Pin, GPIO_PIN_SET);
}


/*
 * @brief Read the IRQ pin (IRQ fired when the pin is LOW).
 */
uint8_t nrfport_getIRQ_B(void)
{
	uint8_t irqFired = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(IRQ_R_GPIO_Port, IRQ_R_Pin)) ? 1 : 0;
	return irqFired;
}
