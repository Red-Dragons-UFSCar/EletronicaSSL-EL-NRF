/*
 * nrf.c
 *
 *  Created on: Aug 15, 2025
 *      Author: udavi
 */

#include "nrf.h"

uint32_t CPU_Freq = 0x00; //Variável para salvar a frequência da CPU

void wait(uint64_t us) {
	uint32_t volatile cycles = CPU_Freq * us / 1000000; //Quantos ciclos de CPU deverão ser esperados para alcancar tal tempo
	uint32_t volatile current = 0; //Quantos se passaram
	while (current <= cycles) {
		current++;
	}
}

void Set_NRF24(NRF24 *nrf, SPI_HandleTypeDef *handle, GPIO_TypeDef *PortCS,
		uint16_t PinCS, GPIO_TypeDef *PortCE, uint16_t PinCE) {
	nrf->spiHandle = handle;
	nrf->csPinBank = PortCS;
	nrf->cePinBank = PortCE;
	nrf->csPin = PinCS;
	nrf->cePin = PinCE;
	nrf->Mode = NRF_Mode_Powerdown;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);// Garante que o Select está em nível alto
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);// Garante que o enable está em nível baixo
	CPU_Freq = HAL_RCC_GetSysClockFreq();

	HAL_Delay(100);
}

NRF_Status NRF24_SendCommand(NRF24 *nrf, uint8_t cmd) {
	NRF_Status ret = NRF_OK;
	uint8_t status;
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET); //csn_reset()
	ret = (NRF_Status) HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
			NRF_SPI_TIMEOUT_TIME); //Envia o comando e retorna o status ********************************************
	if (ret != NRF_OK) { //Verifica se o comando foi enviado corretamente
		return ret;
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET); //csn_set()
	}
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET); //csn_set()
	return ret;
}

NRF_Status NRF24_SendWriteCommand(NRF24 *nrf, uint8_t cmd, uint8_t *write,
		uint8_t length) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET); //cs_reset
	ret = (NRF_Status) HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
	NRF_SPI_TIMEOUT_TIME);
	if(ret != NRF_OK){
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
		return ret;
	}
	ret = (NRF_Status) HAL_SPI_Transmit(nrf->spiHandle, write, length,
	NRF_SPI_TIMEOUT_TIME);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
	return ret;
}

NRF_Status NRF24_SendReadCommand(NRF24 *nrf, uint8_t cmd, uint8_t *read,
		uint8_t length) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET);

	ret = (NRF_Status) HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
	NRF_SPI_TIMEOUT_TIME);
	if(ret != NRF_OK){
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
		return ret;
	}
	ret = (NRF_Status) HAL_SPI_Receive(nrf->spiHandle, read, length,
	NRF_SPI_TIMEOUT_TIME);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
	return ret;
}

NRF_Status NRF24_WriteRegister(NRF24 *nrf, uint8_t reg, uint8_t *write,
		uint8_t Length) {
	return NRF24_SendWriteCommand(nrf, NRF_CMD_W_REGISTER | reg, write, Length);
}

NRF_Status NRF24_WriteRegisterByte(NRF24 *nrf, uint8_t reg, uint8_t byte) {
	uint8_t write = byte;
	return NRF24_WriteRegister(nrf, reg, &write, 1);
}

NRF_Status NRF24_ReadRegister(NRF24 *nrf, uint8_t reg, uint8_t *read,
		uint8_t Length) {
	return NRF24_SendReadCommand(nrf, NRF_CMD_R_REGISTER | reg, read, Length);
}

uint8_t NRF24_ReadRegisterByte(NRF24 *nrf, uint8_t reg) {
	uint8_t read;
	NRF24_SendReadCommand(nrf, NRF_CMD_R_REGISTER | reg, &read, 1);
	return read;
}

NRF_Status NRF24_SetRegisterBit(NRF24 *nrf, uint8_t reg, uint8_t bit) {
	NRF_Status ret = NRF_OK;
	uint8_t cfg = 0x00;

	ret = NRF24_ReadRegister(nrf, reg, &cfg, 1);
	if (ret != NRF_OK) {
		return ret;
	}
	cfg = cfg | (1 << bit);
	return NRF24_WriteRegister(nrf, reg, &cfg, 1);
}

NRF_Status NRF24_ResetRegisterBit(NRF24 *nrf, uint8_t reg, uint8_t bit) {
	NRF_Status ret = NRF_OK;
	uint8_t cfg = 0x00;

	ret = NRF24_ReadRegister(nrf, reg, &cfg, 1); //Pega a informação do byte do registro
	if (ret != NRF_OK) {
		return ret;
	}

	cfg = cfg & ~(1 << bit); //Altera o bit por meio de um E binário
	return NRF24_WriteRegister(nrf, reg, &cfg, 1); //Escreve o registro de volta
}

uint8_t NRF24_ReadStatus(NRF24 *nrf) {
	uint8_t status = 0x00;
	uint8_t cmd = NRF_CMD_NOP;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
	NRF_SPI_TIMEOUT_TIME);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);

	return status;
}

NRF_Status NRF24_EnterMode(NRF24 *nrf, NRF_Mode modo){
	NRF_Status ret = NRF_OK;
	switch(modo){
	case(NRF_Mode_Powerdown):
			HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);
			ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PWR_UP);
			nrf->Mode = NRF_Mode_Powerdown;
			break;
	case(NRF_Mode_Standby):
			if(nrf->Mode == NRF_Mode_Powerdown){
				ret = NRF24_SetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PWR_UP);
				wait(1500);
			} else if(nrf->Mode == NRF_Mode_TxMode){
				HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);
			} else if(nrf->Mode == NRF_Mode_RxMode){
				ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX);
				HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);
			}
			nrf->Mode = NRF_Mode_Standby;
			break;
	case(NRF_Mode_TxMode):
			if(nrf->Mode != NRF_Mode_Standby){
				ret = NRF_BAD_TRANSITION;
			} else {
				ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 0 = Tx
				HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
				wait(10);
				nrf->Mode = NRF_Mode_TxMode;
			}
			break;
	case(NRF_Mode_RxMode):
			if(nrf->Mode != NRF_Mode_Standby){
					ret = NRF_BAD_TRANSITION;
				} else {
					ret = NRF24_SetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 0 = Tx
					HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
					nrf->Mode = NRF_Mode_RxMode;
				}
			break;
	}
	return ret;
}

NRF_Status NRF24_Config(NRF24 *nrf){
	NRF_Status ret = NRF_OK;
	NRF24_EnterMode(nrf, NRF_Mode_Powerdown);
	NRF24_EnterMode(nrf, NRF_Mode_Standby);

	// Flush FIFOs
	NRF24_EnterMode(nrf, NRF_Mode_TxMode);
	NRF24_SendCommand(nrf, NRF_CMD_FLUSH_TX);
	NRF24_EnterMode(nrf, NRF_Mode_Standby);
	NRF24_EnterMode(nrf, NRF_Mode_RxMode);
	NRF24_SendCommand(nrf, NRF_CMD_FLUSH_RX);
	NRF24_EnterMode(nrf, NRF_Mode_Standby);

	NRF24_WriteRegisterByte(nrf, NRF_REG_CONFIG, 0x0A); // 00001010
	NRF24_WriteRegisterByte(nrf, NRF_REG_EN_AA, 0x3F); // 00011111 = AutoAcknologment desligado em todos os Pipes
	NRF24_WriteRegisterByte(nrf, NRF_REG_EN_RXADDR, 0x03); //00000011 -> Pipes 0 e 1 no Rx
	NRF24_WriteRegisterByte(nrf, NRF_REG_SETUP_AW, 0x03); //00000011 -> 5 bytes no adresss
	NRF24_WriteRegisterByte(nrf, NRF_REG_SETUP_RETR, 0x0F); //00000011 -> re-transmit desabilitado
	NRF24_WriteRegisterByte(nrf, NRF_REG_RF_CH, 0x02); //00000010 -> Canal 3
	NRF24_WriteRegisterByte(nrf, NRF_REG_RF_SETUP, 0x0e); //00001110 -> LNA desligado, 0dBm, 2MBs
	NRF24_WriteRegisterByte(nrf, NRF_REG_STATUS, 0x70); // clear flags

	uint8_t address[5] = {1,2,3,4,5};
	uint8_t address2[5] = {2,3,4,5,6};
	NRF24_WriteRegister(nrf, NRF_REG_RX_ADDR_P0, address, 5); //Adress do pipe 0
	NRF24_WriteRegister(nrf, NRF_REG_RX_ADDR_P1, address2, 5); //Adress pipe 1
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_ADDR_P2, 0xC3);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_ADDR_P3, 0xC4);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_ADDR_P4, 0xC5);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_ADDR_P5, 0xC6);
	NRF24_WriteRegister(nrf, NRF_REG_TX_ADDR, address, 5);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P0, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P1, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P2, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P3, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P4, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P5, 0x00);

	NRF24_WriteRegisterByte(nrf, NRF_REG_FIFO_STATUS, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_DYNPD, 0x00);
	NRF24_WriteRegisterByte(nrf, NRF_REG_FEATURE, 0x00);
	return ret;
}

void Tx_mode(NRF24 *nrf, uint8_t Adress[5]) {
	NRF24_Config(nrf);
	NRF24_WriteRegister(nrf, NRF_REG_TX_ADDR, Adress, 5);
	//NRF24_EnterMode(nrf, NRF_Mode_TxMode);
	 //ce_reset();
	//NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P0, dataSize);
	NRF24_WriteRegister(nrf,NRF_REG_RX_ADDR_P0,Adress,5);
	//HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);
}

void Rx_mode(NRF24 *nrf, uint8_t Adress[5], uint8_t dataSize) {
	NRF24_Config(nrf);
	NRF24_WriteRegister(nrf, NRF_REG_RX_ADDR_P0, Adress, 5);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P0, dataSize);
	NRF24_EnterMode(nrf, NRF_Mode_RxMode);
	wait(130);
}

NRF_Status NRF24_WritePayload(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	return NRF24_SendWriteCommand(nrf, NRF_CMD_W_TX_PAYLOAD, payload, length);
}


NRF_Status NRF24_TransmitAndWait(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	NRF_Status ret = NRF_OK;

	ret = NRF24_WritePayload(nrf, payload, length);
	if (ret != NRF_OK) {

		return ret;
	}

	// Transmit
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();

	wait(130);

	// Wait for status update
	uint8_t status;
	for (;;) {
		status = NRF24_ReadStatus(nrf);
		if (status & (1 << STATUS_BIT_TX_DS)) {
			// Packet transmitted
			ret = NRF24_SetRegisterBit(nrf, NRF_REG_STATUS, STATUS_BIT_TX_DS); // clear flag
			break;
		} else if (status & (1 << STATUS_BIT_MAX_RT)) {
			// Max retransmits reached
			NRF24_SetRegisterBit(nrf, NRF_REG_STATUS, STATUS_BIT_MAX_RT); // clear flag
			ret = NRF_MAX_RT;
			break;
		}
	}
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();

	return ret;
}

NRF_Status NRF24_Transmit(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	NRF_Status ret = NRF_OK;
	ret = NRF24_WritePayload(nrf, payload, length);
	if (ret != NRF_OK) {
		return ret;
	}


	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
	wait(130);
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();

	return ret;
}

NRF_Status NRF24_ReadPayload(NRF24 *nrf, uint8_t *read, uint8_t length) {
	return NRF24_SendReadCommand(nrf, NRF_CMD_R_RX_PAYLOAD, read, length);
}

NRF_Status ReceiveData(NRF24 *nrf, uint8_t *data, uint32_t len) {
	NRF_Status ret = NRF_ERROR;
	uint8_t status = NRF24_ReadStatus(nrf);
	uint8_t STATUS_REGISTER_RX_DR_BIT = 6;
	if (status & (1 << STATUS_REGISTER_RX_DR_BIT)) {
		NRF24_ReadPayload(nrf, data, len);
		ret = NRF_OK;
		NRF24_SetRegisterBit(nrf, NRF_REG_STATUS, 6);
	} else {
		ret = NRF_ERROR;
	}
	return ret;
}

NRF_Status NRF24_SendReadCommand_DMA(NRF24 *nrf, uint8_t cmd, uint8_t *read,
		uint8_t length) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET);

	ret = (NRF_Status) HAL_SPI_TransmitReceive_DMA(nrf->spiHandle, &cmd, &status, 1);
	if(ret != NRF_OK){
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
		return ret;
	}
	ret = (NRF_Status) HAL_SPI_Receive_DMA(nrf->spiHandle, read, length);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
	return ret;
}

NRF_Status NRF24_ReadPayload_DMA(NRF24 *nrf, uint8_t *read, uint8_t length) {
	return NRF24_SendReadCommand_DMA(nrf, NRF_CMD_R_RX_PAYLOAD, read, length);
}

NRF_Status ReceiveData_DMA (NRF24 *nrf){
	NRF_Status ret = NRF_ERROR;
	ret = NRF24_ReadPayload_DMA(nrf, nrf->Mensagem, sizeof(nrf->Mensagem));
	NRF24_SetRegisterBit_DMA(nrf, NRF_REG_STATUS, 6);
	return ret;
}

NRF_Status NRF24_SetRegisterBit_DMA(NRF24 *nrf, uint8_t reg, uint8_t bit){
	NRF_Status ret = NRF_OK;
	uint8_t cfg = 0x00;

	ret = NRF24_ReadRegister_DMA(nrf, reg, &cfg, 1); //Pega a informação do byte do registro
		if (ret != NRF_OK) {
			return ret;
		}

		cfg = cfg & ~(1 << bit); //Altera o bit por meio de um E binário
		return NRF24_WriteRegister_DMA(nrf, reg, &cfg, 1); //Escreve o registro de volta

}

NRF_Status NRF24_ReadRegister_DMA(NRF24 *nrf, uint8_t reg, uint8_t *read,
		uint8_t Length) {
	return NRF24_SendReadCommand_DMA(nrf, NRF_CMD_R_REGISTER | reg, read, Length);
}

NRF_Status NRF24_WriteRegister_DMA(NRF24 *nrf, uint8_t reg, uint8_t *write,
		uint8_t Length) {
	return NRF24_SendWriteCommand_DMA(nrf, NRF_CMD_W_REGISTER | reg, write, Length);
}

NRF_Status NRF24_SendWriteCommand_DMA(NRF24 *nrf, uint8_t cmd, uint8_t *write,
		uint8_t length) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET); //cs_reset
	ret = (NRF_Status) HAL_SPI_TransmitReceive_DMA(nrf->spiHandle, &cmd, &status, 1);
	if(ret != NRF_OK){
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
		return ret;
	}
	ret = (NRF_Status) HAL_SPI_Transmit_DMA(nrf->spiHandle, write, length);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);
	return ret;
}
