/*
 * nrf24l01p_driver.c
 *
 *  Created on: Oct 31, 2022
 *      Author: Vladimir Sustek
 */


/**
  * @brief  NRF24L01+ driver module. All functionalities covered.
  *
  *
  * How to use the driver:
  *
  * PTX - Primary transmitter
  * PRX - Primary receiver
  * STATUS - the NRF240l+ STATUS register (address 0x07)
  *
  * 0) Read datasheet for NRF24L01+ for better module understanding.
  *
  * 1) Call the NRF_initHostHW() to initialize SPI, CE GPIO and IRQ.
  *
  * 2) Configure all registers of NRF24L01+ to become a receiver (PRX) or
  * a transmitter (PTX) via NRF_configure(PTX) or NRF_configure(PRX).
  * By default PRX and PTX are configured to have address "ADR01" and
  * use ERX_P0. The "ACK with payload" mode is used, so when PRX
  * receives from PTX, the PRX sends back an ACK with payload.
  *
  * 3) PTX: Upload payload to be transmitted via NRF_setW_TX_PAYLOAD().
  *    PRX: Upload payload to be transmitted via NRF_set_W_ACK_PAYLOAD().
  *
  *    note that max PAYLOAD length is 32byte.
  *
  * 4) Activate RF via NRF_CEactivate() to send or start listen (receive).
  * When both modules are on one host, PRX must "listen" before PTX transmits!
  *
  * During NRF_CEactivate() is the PTX's payload transmitted into the the PRX
  * and the PRX sends back ACK with payload.
  *
  * In successful case the PRX's IRQ fires, the STATUS bit RX_DR is set as well as
  * RX_P_NO_2 - RX_P_NO_0 signalizes RX_PIPE (ERX_P0 means the RX_P_NO_2 - RX_P_NO_0
  * and shall be 0). Reading any of the NRF's IRQs is possible via NRF_getIRQ().
  * Also the PRX automatically sends ACK with payload back to the PTX.
  * The PRX is capable to read the payload, it's length and provide it to the host.
  *
  * Note that the RX_DR stands for "received data flag".
  *
  * Meantime, the PTX's IRQ fires, the STATUS TX_DS bit is set as well as the
  * RX_DR bit + the RX_P_NO_2 - RX_P_NO_0 signalizes RX_PIPE (Pipe is 0). The
  * PTX is capable to read the ack payload, it's length and provide the ack payload
  * to the host. The PTX does not send anything back to the PRX (no 2nd ACK exist),
  * so the PRX is unaware whether the ack arrived.
  *
  * Note that the TX_DS stands for "Data send, ACK received" flag.
  *
  * All set status bits must be cleared to allow further operation as well as both
  * payloads must be read to clear RX_P_NO_x bits:
  *
  * clear bits using:
  * NRF_setStatus(1 << TX_DS)
  * NRF_setStatus(1 << RX_DR)
  *
  * read payload (or ack payload) using:
  * pld_lng = NRF_getR_RX_PLD_WID()
  * NRF_getR_RX_PAYLOAD(some_uint8_t_buffer, pld_lng)
  *
  * When the PRX does not receive anything its state does not change nor the IRQ fires.
  * However when the PTX tried to send and did not get an ACK, the MAX_RT bit in the
  * STATUS is set (also IRQ fires) as the receiver was not reached. In this case the
  * PTX may reuse already uploaded payload with NRF_setREUSE_TX_PL() and try to send
  * the payload back. The PRX's state did not change and needs no action.
  *
  * When the PTX's payload shall not be used again, must be flushed via NRF_setFLUSH_TX()
  * and then a new payload may be uploaded. Also the reicever does not have to read the
  * payload and may just flush it via the NRF_setFLUSH_RX().
  *
  * There is also a posibility to send just an ACK without payload from the PRX to the
  * PTX, but this must be reflected in the functions NRF_configure(). for the both PTX and
  * PRX. The FEATURE register EN_ACK_PAY bit must be 0 .
  * In this case calling the NRF_set_W_ACK_PAYLOAD() on the PRX side makes no sense.
  * During this operation only an ACK arrives (no payload), the PTX's STATUS does not
  * have theRX_DR set to 1 as well as RX_P_NO_2 - RX_P_NO_1 stays default. PRX side
  * STATUS bits have no difference in compare to the ack + payload case.
  *
  * The last possibility is to don't use ACK at all (PRX will not send anything back
  * once it receives a message). This case needs also changes in NRF_configure()
  * The EN_AA register bit ENAA_P0 set to 0. (ENAA_P0 because the PIPE 0 is used).
  * The PTX must upload a payload via the NRF_setW_TX_PAYLOAD_NO_ACK().
  *
  * 5) After correct handling the states, possible to infinitely repeat steps 3 - 4.
  *
  */

#include <string.h>

#include "nrf24l01p_driver_B.h"
#include "nrf24l01p_port_B.h"


/**
  * @brief Initialize host to allow NRF24L01+ operate.
  */
uint8_t NRF_initHostHW_B()
{
	return nrfport_initHostHW_B();
}



/**
 * @brief Configure NRF24L01+ for ACK+payload P2P operation.
 *
 * @param ptx_nprx when true configures PTX, when false configures PRX
 *
 * @note PRX is primary receiver, PTX is primary transmitter.
 * The device will use dynamic payload length, auto-acknowledgement with
 * payload, maximal RF power and maximal number of re-transmit retries
 * as well as maximal rx timeout. At the end RX and TX FIFO are flushed.
 *
 * See datasheet of the NRF24L01+ to extend or change configuring routine.
 */
void NRF_configure_B(bool ptx_nprx)
{
	const uint8_t config_prx = (1 << EN_CRC | 1 << CRCO | 1 << PWR_UP | 1 << PRIM_RX);
	const uint8_t config_ptx = (1 << EN_CRC | 1 << CRCO | 1 << PWR_UP);

	const uint8_t en_aa = (1 << ENAA_P0);
	const uint8_t en_rxaddr = (1 << ERX_P0);
	const uint8_t setup_aw = (1 << AW_1 | 1 << AW_0);

	const uint8_t setup_retr = (1 << ARD_3 | 1 << ARD_2 | 1 << ARD_1 | 1 << ARD_0 |
				   1 << ARC_3 | 1 << ARC_2 | 1 << ARC_1 | 1 << ARC_0);

	const uint8_t rf_ch = (1 << RF_CH_1);
	const uint8_t rf_setup = (1 << RF_DR_HIGH | 1 << RF_PWR_1 | 1 << RF_PWR_0);
	const uint8_t status = (1 << RX_DR | 1 << TX_DS | 1 << MAX_RT | 1 << TX_FULL);
	const uint8_t dynpd = (1 << DPL_P0);
	const uint8_t feature = (1 << EN_DPL | 1 << EN_ACK_PAY);


	const char rx_adr_p0[NRF_ADR_MAX] = {"ADR01"};
	const char tx_adr[NRF_ADR_MAX] = {"ADR01"};

	if(ptx_nprx)
	{
		NRF_setCONFIG_B(config_ptx);
	}
	else
	{
		NRF_setCONFIG_B(config_prx);
	}

	NRF_setEN_AA_B(en_aa);
	NRF_setEN_RXADDR_B(en_rxaddr);
	NRF_setSETUP_AW_B(setup_aw);
	NRF_setSETUP_RETR_B(setup_retr);
	NRF_setRF_CH_B(rf_ch);
	NRF_setRF_SETUP_B(rf_setup);
	NRF_setSTATUS_B(status);
	NRF_setDYNPD_B(dynpd);
	NRF_setFEATURE_B(feature);

	if(ptx_nprx)
	{
		NRF_setTX_ADDR_B((uint8_t*)tx_adr, NRF_ADR_MAX);
	}

	NRF_setRX_ADDR_P0_B((uint8_t*)rx_adr_p0, NRF_ADR_MAX);

	NRF_setFLUSH_RX_B();
	NRF_setFLUSH_TX_B();
}

/**
  * @brief Get the CONFIG register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getCONFIG_B(void)
{
	return nrfport_readByte_B(CONFIG_ADR);
}


/**
  * @brief Get the EN_AA register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getEN_AA_B(void)
{
	return nrfport_readByte_B(EN_AA_ADR);
}


/**
  * @brief Get the EN_RXADDR register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getEN_RXADDR_B(void)
{
	return nrfport_readByte_B(EN_RXADDR_ADR);
}


/**
  * @brief Get the SETUP_AW register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getSETUP_AW_B(void)
{
	return nrfport_readByte_B(SETUP_AW_ADR);
}


/**
  * @brief Get the SETUP_RETR register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getSETUP_RETR_B(void)
{
	return nrfport_readByte_B(SETUP_RETR_ADR);
}


/**
  * @brief Get the RF_CH register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRF_CH_B(void)
{
	return nrfport_readByte_B(RF_CH_ADR);
}


/**
  * @brief Get the SETUP register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRF_SETUP_B(void)
{
	return nrfport_readByte_B(RF_SETUP_ADR);
}


/**
  * @brief Get the STATUS register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getSTATUS_B(void)
{
	return nrfport_readByte_B(STATUS_ADR);
}


/**
  * @brief Get the OBSERVE_TX register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getOBSERVE_TX_B(void)
{
	return nrfport_readByte_B(OBSERVE_TX_ADR);
}


/**
  * @brief Get the RPD register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRPD_B(void)
{
	return nrfport_readByte_B(RPD_ADR);
}


/**
  * @brief Get the RX_ADDR_P0 register.
  *
  * @param arg buffer to read the address RX_ADDR_P0.
  * @param lng length of the address (3 - 5 byte)
  */
void NRF_getRX_ADDR_P0_B(uint8_t* arg, uint32_t lng)
{
	if(NULL == arg || lng < NRF_ADR_MIN || lng > NRF_ADR_MAX) return;

	uint8_t tx[RD_RX_ADR_P0_LNG] = {0};
	uint8_t rx[RD_RX_ADR_P0_LNG] = {0};

	tx[RD_BYTE_CMD] = CMD_R_REGISTER | RX_ADR_P0_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);

	memcpy(arg, rx + 1, lng);

}


/**
  * @brief Get the RX_ADDR_P1 register.
  *
  * @param arg buffer to read the address RX_ADDR_P1.
  * @param lng length of the address (3 - 5 byte).
  */
void NRF_getRX_ADDR_P1_B(uint8_t* arg, uint32_t lng)
{
	if(NULL == arg || lng < NRF_ADR_MIN || lng > NRF_ADR_MAX) return;

	uint8_t tx[RD_RX_ADR_P0_LNG] = {0};
	uint8_t rx[RD_RX_ADR_P0_LNG] = {0};

	tx[RD_BYTE_CMD] = CMD_R_REGISTER | RX_ADR_P0_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);

	memcpy(arg, rx + 1, lng);
}


/**
  * @brief Get the RX_ADDR_P2 register.
  *
  * @note only last RX_ADDR_P2 byte is different than RX_ADDR_P1.
  *
  * @return 8-bit of the last address byte
  */
uint8_t NRF_getRX_ADDR_P2_B(void)
{
	return nrfport_readByte_B(RX_ADR_P2_ADR);
}


/**
  * @brief Get the RX_ADDR_P3 register.
  *
  * @note only last RX_ADDR_P3 byte is different than RX_ADDR_P1.
  *
  * @return 8-bit of the last address byte
  */
uint8_t NRF_getRX_ADDR_P3_B(void)
{
	return nrfport_readByte_B(RX_ADR_P3_ADR);
}


/**
  * @brief Get the RX_ADDR_P4 register.
  *
  * @note only last RX_ADDR_P4 byte is different than RX_ADDR_P1.
  *
  * @return 8-bit of the last address byte
  */
uint8_t NRF_getRX_ADDR_P4_B(void)
{
	return nrfport_readByte_B(RX_ADR_P4_ADR);
}


/**
  * @brief Get the RX_ADDR_P5 register.
  *
  * @note only last RX_ADDR_P5 byte is different than RX_ADDR_P1.
  *
  * @return 8-bit of the last address byte
  */
uint8_t NRF_getRX_ADDR_P5_B(void)
{
	return nrfport_readByte_B(RX_ADR_P5_ADR);
}


/**
  * @brief Get the TX_ADDR register.
  *
  * @param buffer to read the address TX_ADDR.
  * @param lng length of the address (3 - 5 byte)
  */
void NRF_getTX_ADDR_B(uint8_t* arg, uint32_t lng)
{
	if(NULL == arg || lng < NRF_ADR_MIN || lng > NRF_ADR_MAX) return;

	uint8_t tx[RD_RX_ADR_P0_LNG] = {0};
	uint8_t rx[RD_RX_ADR_P0_LNG] = {0};

	tx[RD_BYTE_CMD] = CMD_R_REGISTER | RX_ADR_P0_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);

	memcpy(arg, rx + 1, lng);
}


/**
  * @brief Get the RX_PW_P0 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P0_B(void)
{
	return nrfport_readByte_B(RX_PW_P0_ADR);
}


/**
  * @brief Get the RX_PW_P1 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P1_B(void)
{
	return nrfport_readByte_B(RX_PW_P1_ADR);
}


/**
  * @brief Get the RX_PW_P2 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P2_B(void)
{
	return nrfport_readByte_B(RX_PW_P2_ADR);
}


/**
  * @brief Get the RX_PW_P3 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P3_B(void)
{
	return nrfport_readByte_B(RX_PW_P3_ADR);
}


/**
  * @brief Get the RX_PW_P4 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P4_B(void)
{
	return nrfport_readByte_B(RX_PW_P4_ADR);
}


/**
  * @brief Get the RX_PW_P5 register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getRX_PW_P5_B(void)
{
	return nrfport_readByte_B(RX_PW_P5_ADR);
}


/**
  * @brief Get the FIFO_STATUS register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getFIFO_STATUS_B(void)
{
	return nrfport_readByte_B(FIFO_STATUS_ADR);
}


/**
  * @brief Get the DYNPD register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getDYNPD_B(void)
{
	return nrfport_readByte_B(DYNPD_ADR);
}


/**
  * @brief Get the FEATURE register.
  *
  * @return 8-bit register content.
  */
uint8_t NRF_getFEATURE_B(void)
{
	return nrfport_readByte_B(FEATURE_ADR);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setCONFIG_B(uint8_t arg)
{
	nrfport_writeByte_B(CONFIG_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setEN_AA_B(uint8_t arg)
{
	nrfport_writeByte_B(EN_AA_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setEN_RXADDR_B(uint8_t arg)
{
	nrfport_writeByte_B(EN_RXADDR_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setSETUP_AW_B(uint8_t arg)
{
	nrfport_writeByte_B(SETUP_AW_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setSETUP_RETR_B(uint8_t arg)
{
	nrfport_writeByte_B(SETUP_RETR_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setRF_CH_B(uint8_t arg)
{
	nrfport_writeByte_B(RF_CH_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setRF_SETUP_B(uint8_t arg)
{
	nrfport_writeByte_B(RF_SETUP_ADR, arg);
}


/**
  * @brief Set the CONFIG register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setSTATUS_B(uint8_t arg)
{
	nrfport_writeByte_B(STATUS_ADR, arg);
}


/**
  * @brief Set the OBSERVE_TX register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setOBSERVE_TX_B(uint8_t arg)
{
	nrfport_writeByte_B(OBSERVE_TX_ADR, arg);
}


/**
  * @brief Set the PRD register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setRPD_B(uint8_t arg)
{
	nrfport_writeByte_B(RPD_ADR, arg);
}


/**
  * @brief Set the RX_ADDR_P0.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  */
void NRF_setRX_ADDR_P0_B(uint8_t* arg, uint32_t lng)
{
	if (lng < NRF_ADR_MIN || lng > NRF_ADR_MAX || arg == NULL)
		return;

	uint8_t tx[WR_RX_ADR_P0_LNG] = {0};
	uint8_t rx[WR_RX_ADR_P0_LNG] = {0};

	memcpy(tx + 1, arg, lng);

	tx[RD_BYTE_CMD] = CMD_W_REGISTER | RX_ADR_P0_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);
}


/**
  * @brief Set the RX_ADDR_P1.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  */
void NRF_setRX_ADDR_P1_B(uint8_t* arg, uint32_t lng)
{
	if (lng < NRF_ADR_MIN || lng > NRF_ADR_MAX || arg == NULL)
		return;

	uint8_t tx[WR_RX_ADR_P0_LNG] = {0};
	uint8_t rx[WR_RX_ADR_P0_LNG] = {0};

	memcpy(tx + 1, arg, lng);

	tx[RD_BYTE_CMD] = CMD_W_REGISTER | RX_ADR_P0_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);
}


/**
  * @brief Set the RX_ADDR_P2.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  * @note only last RX_ADDR_P2 byte is different than RX_ADDR_P1.
  *
  */
void NRF_setRX_ADDR_P2_B(uint8_t arg)
{
	nrfport_writeByte_B(RX_ADR_P2_ADR, arg);
}


/**
  * @brief Set the RX_ADDR_P3.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  * @note only last RX_ADDR_P3 byte is different than RX_ADDR_P1.
  *
  */
void NRF_setRX_ADDR_P3_B(uint8_t arg)
{
	nrfport_writeByte_B(RX_ADR_P3_ADR, arg);
}


/**
  * @brief Set the RX_ADDR_P4.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  * @note only last RX_ADDR_P4 byte is different than RX_ADDR_P1.
  *
  */
void NRF_setRX_ADDR_P4_B(uint8_t arg)
{
	nrfport_writeByte_B(RX_ADR_P4_ADR, arg);
}


/**
  * @brief Set the RX_ADDR_P5.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  * @note only last RX_ADDR_P5 byte is different than RX_ADDR_P1.
  *
  */
void NRF_setRX_ADDR_P5_B(uint8_t arg)
{
	nrfport_writeByte_B(RX_ADR_P5_ADR, arg);
}


/**
  * @brief Set the TX_ADDR.
  *
  * @param arg buffer where address is stored.
  * @param lng of the address (3-5).
  *
  */
void NRF_setTX_ADDR_B(uint8_t* arg, uint32_t lng)
{
	if (lng < NRF_ADR_MIN || lng > NRF_ADR_MAX || arg == NULL)
		return;

	uint8_t tx[WR_RX_ADR_P0_LNG] = {0};
	uint8_t rx[WR_RX_ADR_P0_LNG] = {0};

	memcpy(tx + 1, arg, lng);

	tx[RD_BYTE_CMD] = CMD_W_REGISTER | TX_ADR_ADR;

	nrfport_spiTransaction_B(tx, rx, lng + 1);
}

/**
  * @brief Set the FIFO_STATUS register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setFIFO_STATUS_B(uint8_t arg)
{
	nrfport_writeByte_B(FIFO_STATUS_ADR, arg);
}


/**
  * @brief Set the DYNPD register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setDYNPD_B(uint8_t arg)
{
	nrfport_writeByte_B(DYNPD_ADR, arg);
}


/**
  * @brief Set the FEATURE register.
  *
  * @param arg 8-bit register content to be set.
  */
void NRF_setFEATURE_B(uint8_t arg)
{
	nrfport_writeByte_B(FEATURE_ADR, arg);
}


/**
  * @brief Get received payload R_RX_PAYLOAD.
  *
  * @param arg buffer where the payload will be copied.
  * @param length of data to be copied.
  *
  * @note arg size must be as big as length specifies.
  * Maximal possible payload length is 32 byte.
  */
void NRF_getR_RX_PAYLOAD_B(uint8_t* arg, uint32_t lng)
{
	if(NULL == arg || lng > PAYLOAD_MAX || lng < PAYLOAD_MIN) return;

	uint8_t tx[33] = {0};
	uint8_t rx[33] = {0};

	tx[RD_BYTE_CMD] = CMD_R_RX_PAYLOAD;
	nrfport_spiTransaction_B(tx, rx, lng + 1);
	memcpy(arg, rx + 1, lng);
}


/**
  * @brief Set payload for PTX to be transmitted.
  *
  * @param arg buffer where from the payload will be copied.
  * @param length of data to be copied.
  *
  * @note arg size must be as big as length specifies.
  * Maximal possible payload length is 32 byte.
  */
void NRF_setW_TX_PAYLOAD_B(uint8_t* arg, uint32_t lng)
{
	if(lng > PAYLOAD_MAX || lng < PAYLOAD_MIN || arg == NULL) return;

	uint8_t tx[PAYLOAD_MAX + 1] = {0};
	uint8_t rx[PAYLOAD_MAX + 1] = {0};

	tx[RD_BYTE_CMD] = CMD_W_TX_PAYLOAD;
	memcpy(tx+1, arg, lng);
	nrfport_spiTransaction_B(tx, rx, lng+1);
}


/**
  * @brief Flush all payloads in the TX FIFO.
  */
void NRF_setFLUSH_TX_B(void)
{
	uint8_t tx = CMD_FLUSH_TX, rx = 0;
	nrfport_spiTransaction_B(&tx, &rx, sizeof(tx));
}


/**
  * @brief Flush all payloads in the RX FIFO.
  */
void NRF_setFLUSH_RX_B(void)
{
	uint8_t tx = CMD_FLUSH_RX, rx = 0;
	nrfport_spiTransaction_B(&tx, &rx, sizeof(tx));
}


/**
  * @brief Reuse paylod for PTX (when was not successfully sent).
  */
void NRF_setREUSE_TX_PL_B(void)
{
	uint8_t tx = CMD_REUSE_TX_PL, rx = 0;
	nrfport_spiTransaction_B(&tx, &rx, sizeof(tx));
}


/**
  * @brief Get width of the received payload.
  *
  * @return 8-bit value of the width (0-32 bytes).
  */
uint8_t NRF_getR_RX_PL_WID_B(void)
{
	return nrfport_readByte_B(CMD_R_RX_PL_WID);
}


/**
  * @brief Set payload for PRX to be transmitted within the ACK.
  *
  * @param pipe the pipe number where the ACK and payload shall be sent
  * @param arg buffer where from the payload will be copied.
  * @param length of data to be copied.
  *
  * @note arg size must be as big as length specifies.
  * Maximal possible payload length is 32 byte.
  */
void NRF_set_W_ACK_PAYLOAD_B(uint8_t pipe, uint8_t* arg, uint32_t lng)
{
	if(lng > PAYLOAD_MAX || lng < PAYLOAD_MIN || arg == NULL) return;

	uint8_t tx[PAYLOAD_MAX + 1] = {0};
	uint8_t rx[PAYLOAD_MAX + 1] = {0};

	tx[RD_BYTE_CMD] = CMD_W_TX_PAYLOAD|pipe;
	memcpy(tx+1, arg, lng);
	nrfport_spiTransaction_B(tx, rx, lng+1);
}


/**
  * @brief Set payload for PTX to be transmitted when no ack mode used.
  *
  * @param arg buffer where from the payload will be copied.
  * @param length of data to be copied.
  *
  * @note arg size must be as big as length specifies.
  * Maximal possible payload length is 32 byte.
  */
void NRF_setW_TX_PAYLOAD_NO_ACK_B(uint8_t* arg, uint32_t lng)
{
	if(lng > PAYLOAD_MAX || lng < PAYLOAD_MIN || arg == NULL) return;

	uint8_t tx[PAYLOAD_MAX + 1] = {0};
	uint8_t rx[PAYLOAD_MAX + 1] = {0};

	tx[RD_BYTE_CMD] = CMD_W_TX_PAYLOAD;
	memcpy(tx+1, arg, lng);
	nrfport_spiTransaction_B(tx, rx, lng+1);
}


/**
  * @brief Empty instruction useful for reading the STATUS register.
  *
  * @return 8-bit value of the STATUS register content .
  */
uint8_t NRF_getNOP_B(void)
{
	uint8_t tx = CMD_NOP, rx = 0;
	nrfport_spiTransaction_B(&tx, &rx, sizeof(tx));
	return rx;
}


/**
  * @brief Get interrupt flag. Shall be used by application to read whether IRQ fired.
  *
  * @param clear the interrupt when is asserted.
  */
uint8_t NRF_getIRQ_B(uint8_t clear)
{
	return nrfport_getIRQ_B();
}


/**
  * @brief Activate CE pin of the NRF24L01+.
  *
  * @note This causes transmit (PTX) or receive (PRX).
  * During the CE high the NRF24L01+ has way higher
  * current consumption as the RF module is active.
  */
void NRF_CEactivate_B(void)
{
	nrfport_ceHigh_B();
}


/**
  * @brief De-activate CE pin of the NRF24L01+.
  *
  * @note This returns from transmit (PTX) or receive mode (PRX).
  * During the CE low the NRF24L01+ consumption is way smaller
  * than for CE high case as the RF module is deactivate.
  */
void NRF_CEdeactivate_B(void)
{
	nrfport_ceLow_B();
}


/**
 * @brief Power up the device (power supply line).
 */
void NRF_powerUp_B(void)
{
	nrfport_powerUp_B();
}



/**
 * @brief Power down the device (power supply line).
 */
void NRF_powerDown_B(void)
{
	nrfport_powerDown_B();
}
