/*
 * nrf24l01p_port_B.h
 *
 *  Created on: Nov 19, 2022
 *      Author: 42077
 */

#ifndef NRF24L01P_PORT_B_H_
#define NRF24L01P_PORT_B_H_

uint8_t nrfport_initHostHW_B();
void nrfport_spiTransaction_B(uint8_t* tx, uint8_t *rx, uint32_t lng);
void nrfport_writeByte_B(uint8_t adr, uint8_t data);
uint8_t nrfport_readByte_B(uint8_t adr);
void nrfport_ceHigh_B(void);
void nrfport_ceLow_B(void);
void nrfport_powerUp_B(void);
void nrfport_powerDown_B(void);
uint8_t nrfport_getIRQ_B(void);

#endif /* NRF24L01P_PORT_B_H_ */
