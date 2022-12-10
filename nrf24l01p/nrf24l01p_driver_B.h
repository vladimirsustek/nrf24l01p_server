/*
 * nrf24l01p_driver_B.h
 *
 *  Created on: Nov 13, 2022
 *      Author: 42077
 */

#ifndef NRF24L01P_DRIVER_B_H_
#define NRF24L01P_DRIVER_B_H_

#include "nrf24l01p_defines.h"
#include "string.h"


uint8_t NRF_initHostHW_B();
void NRF_configure_B(bool ptx_nprx);

 uint8_t NRF_getCONFIG_B(void);
 uint8_t NRF_getEN_AA_B(void);
 uint8_t NRF_getEN_RXADDR_B(void);
 uint8_t NRF_getSETUP_AW_B(void);
 uint8_t NRF_getSETUP_RETR_B(void);
 uint8_t NRF_getRF_CH_B(void);
 uint8_t NRF_getRF_SETUP_B(void);
 uint8_t NRF_getSTATUS_B(void);
 uint8_t NRF_getOBSERVE_TX_B(void);
 uint8_t NRF_getRPD_B(void);
 void NRF_getRX_ADDR_P0_B(uint8_t* arg, uint32_t lng);
 void NRF_getRX_ADDR_P1_B(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getRX_ADDR_P2_B(void);
 uint8_t NRF_getRX_ADDR_P3_B(void);
 uint8_t NRF_getRX_ADDR_P4_B(void);
 uint8_t NRF_getRX_ADDR_P5_B(void);
 void NRF_getTX_ADDR_B(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getRX_PW_P0_B(void);
 uint8_t NRF_getRX_PW_P1_B(void);
 uint8_t NRF_getRX_PW_P2_B(void);
 uint8_t NRF_getRX_PW_P3_B(void);
 uint8_t NRF_getRX_PW_P4_B(void);
 uint8_t NRF_getRX_PW_P5_B(void);
 uint8_t NRF_getFIFO_STATUS_B(void);
 uint8_t NRF_getDYNPD_B(void);
 uint8_t NRF_getFEATURE_B(void);

 void NRF_setCONFIG_B(uint8_t arg);
 void NRF_setEN_AA_B(uint8_t arg);
 void NRF_setEN_RXADDR_B(uint8_t arg);
 void NRF_setSETUP_AW_B(uint8_t arg);
 void NRF_setSETUP_RETR_B(uint8_t arg);
 void NRF_setRF_CH_B(uint8_t arg);
 void NRF_setRF_SETUP_B(uint8_t arg);
 void NRF_setSTATUS_B(uint8_t arg);
 void NRF_setOBSERVE_TX_B(uint8_t arg);
 void NRF_setRPD_B(uint8_t arg);
 void NRF_setRX_ADDR_P0_B(uint8_t* arg, uint32_t lng);
 void NRF_setRX_ADDR_P1_B(uint8_t* arg, uint32_t lngs);
 void NRF_setRX_ADDR_P2_B(uint8_t arg);
 void NRF_setRX_ADDR_P3_B(uint8_t arg);
 void NRF_setRX_ADDR_P4_B(uint8_t arg);
 void NRF_setRX_ADDR_P5_B(uint8_t arg);
 void NRF_setTX_ADDR_B(uint8_t* arg, uint32_t lng);
 void NRF_setFIFO_STATUS_B(uint8_t arg);
 void NRF_setDYNPD_B(uint8_t arg);
 void NRF_setFEATURE_B(uint8_t arg);

 void NRF_getR_RX_PAYLOAD_B(uint8_t* arg, uint32_t lng);
 void NRF_setW_TX_PAYLOAD_B(uint8_t* arg, uint32_t lng);
 void NRF_setFLUSH_TX_B(void);
 void NRF_setFLUSH_RX_B(void);
 void NRF_setREUSE_TX_PL_B(void);
 uint8_t NRF_getR_RX_PL_WID_B(void);
 void NRF_set_W_ACK_PAYLOAD_B(uint8_t pipe, uint8_t* arg, uint32_t lng);
 void NRF_setW_TX_PAYLOAD_NO_ACK_B(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getNOP_B(void);
 void NRF_CEactivate_B(void);
 void NRF_CEdeactivate_B(void);

 uint8_t NRF_getIRQ_B();

 void NRF_powerUp_B(void);
 void NRF_powerDown_B(void);

 uint32_t NRF_activeRF_B(uint32_t (*msTickGet)(), void (*msDelay)(uint32_t), uint32_t timeOut);
 uint8_t NRF_postProcess_B(uint8_t pipe, uint8_t* rxBuff);
 uint32_t NRF_powerCycle_B(void (*msDelay)(uint32_t));
#endif /* NRF24L01P_DRIVER_B_H_ */
