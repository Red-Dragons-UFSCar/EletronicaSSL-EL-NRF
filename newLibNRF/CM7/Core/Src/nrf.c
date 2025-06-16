/*
 * NRF24L01.c
 *
 * Created on: March 17, 2025
 * 	Author: Mancoo
 */

#include "nrf.h"
//*******************************************************************************************************************************************************************************
// Bits de configuração
#define CFG_BIT_EN_CRC  0x3 //Posicção do Bit de CRC no registro de power do NRF
#define CFG_BIT_PWR_UP  0x1 //Posição do Bit de Power up no registro de power do NRF
#define CFG_BIT_PRIM_RX 0x0 //Posição do Bit de Rx/Tx no registro de power do NRF (0 = Tx, 1 = Rx) <- VALOR NO REGISTRO NAO POSIÇÃO
//define dos estados do nrf
#define STATUS_BIT_RX_DR   0x6
#define STATUS_BIT_TX_DS   0x5
#define STATUS_BIT_MAX_RT  0x4
#define STATUS_BIT_RX_P_NO 0x1 // consists of bits 1-3
#define STATUS_BIT_TX_FULL 0x0

/* Local globals */
SPI_HandleTypeDef *HSPI; //SPI a ser utilizada
GPIO_TypeDef *NRF_CSN_Port; //Porta do CS do NRF
uint16_t NRF_CSN_Pin; //Pino do CS do NRF
GPIO_TypeDef *NRF_CE_Port; //Porta do CE do NRF
uint16_t NRF_CE_Pin;//Pino do CE do NRF
uint32_t CPU_Freq = 0x00; //Variável para salvar a frequência da CPU
int current_mode = NRF_MODE_POWERDOWN; // Modo atual do NRF


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
	nrf->mode = NRF_MODE_POWERDOWN;
}

//nao sei se vc esqueceu ou tinha a intencao de mudar msm mas coloquei
NRF_Status NRF24_SendCommand(NRF24 *nrf, uint8_t cmd) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET); //csn_reset()
	ret = (NRF_Status) HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
			NRF_SPI_TIMEOUT); //Envia o comando e retorna o status ********************************************
	if (ret != NRF_OK) { //Verifica se o comando foi enviado corretamente
		return ret;
	}
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET); //csn_set()

	return ret;
}

NRF_Status NRF24_SendWriteCommand(NRF24 *nrf, uint8_t cmd, uint8_t *write,
		uint8_t length) {
	NRF_Status ret = NRF_OK;
	uint8_t status;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET); //seleciona o dispositivo
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

uint8_t NRF24_ReadStatus(NRF24 *nrf) {
	uint8_t status = 0x00;
	uint8_t cmd = NRF_CMD_NOP;

	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(nrf->spiHandle, &cmd, &status, 1,
	NRF_SPI_TIMEOUT_TIME);
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);

	return status;
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

//eu mudei isto ent acho que tem erros kkk

NRF_Status NRF24_EnterMode(NRF24 *nrf, uint8_t mode) {
	NRF_Status ret = NRF_OK;

	switch (mode) {
	case NRF_MODE_POWERDOWN:
		HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET); //csn_set();
		HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();
		ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PWR_UP); //Seta o bit de Power up em 0 *****************************************8
		break;

	case NRF_MODE_STANDBY1:
		if (nrf->mode == NRF_MODE_POWERDOWN) {
			ret = NRF24_SetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PWR_UP); //Seta o bit de Power up em 1
			wait(1500);
		} else if (nrf->mode == NRF_MODE_RX) {
			ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX);
			HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();
		} else if (nrf->mode == NRF_MODE_TX) {
			HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET);
		}
		break;

	case NRF_MODE_RX:
		if (nrf->mode != NRF_MODE_STANDBY1) { //O dispositivo deve estar em standby para passar para o modo Rx
			return NRF_BAD_TRANSITION;
		}
		ret = NRF24_SetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 1 = Rx
		HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
		break;

	case NRF_MODE_TX:
		if (nrf->mode != NRF_MODE_STANDBY1) {
			return NRF_BAD_TRANSITION;
		}
		ret = NRF24_ResetRegisterBit(nrf, NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 0 = Tx
		HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET);
		break;

	default:
		ret = NRF_ERROR;
		break;
	}

	if (ret == NRF_OK) {
		nrf->mode = mode;
	}

	return ret;
}

NRF_Status NRF24_Init(NRF24 *nrf, SPI_HandleTypeDef *handle,
		GPIO_TypeDef *PortCS, uint16_t PinCS, GPIO_TypeDef *PortCE,
		uint16_t PinCE) {

	CPU_Freq = HAL_RCC_GetSysClockFreq();
	if (CPU_Freq == 0x00) {
		return NRF_ERROR;
	}

	// Make sure CSN is pulled high
	HAL_GPIO_WritePin(nrf->csPinBank, nrf->csPin, GPIO_PIN_SET);

	// Takes ~100ms from power on to start up
	HAL_Delay(100);

	return NRF24_EnterMode(nrf, NRF_MODE_STANDBY1);
}

void NRF24_Reset(NRF24 *nrf) {
	NRF24_EnterMode(nrf, NRF_MODE_POWERDOWN);
	NRF24_EnterMode(nrf, NRF_MODE_STANDBY1);

	// Flush FIFOs
	NRF24_EnterMode(nrf, NRF_MODE_TX);
	NRF24_SendCommand(nrf, NRF_CMD_FLUSH_TX);
	NRF24_EnterMode(nrf, NRF_MODE_STANDBY1);
	NRF24_EnterMode(nrf, NRF_MODE_RX);
	NRF24_SendCommand(nrf, NRF_CMD_FLUSH_RX);
	NRF24_EnterMode(nrf, NRF_MODE_STANDBY1);

	// Flush register -> LER DATASHEET!!!!!!!!!!!!!!
	NRF24_WriteRegisterByte(nrf, NRF_REG_CONFIG, 0x0A); // 00001010
	NRF24_WriteRegisterByte(nrf, NRF_REG_EN_AA, 0x03); // 00000000 = AutoAcknologment desligado em todos os Pipes
	NRF24_WriteRegisterByte(nrf, NRF_REG_EN_RXADDR, 0x03); //00000011 -> Pipes 0 e 1 no Rx
	NRF24_WriteRegisterByte(nrf, NRF_REG_SETUP_AW, 0x03); //00000011 -> 5 bytes no adresss
	NRF24_WriteRegisterByte(nrf, NRF_REG_SETUP_RETR, 0x03); //00000000 -> re-transmit desabilitado
	NRF24_WriteRegisterByte(nrf, NRF_REG_RF_CH, 0x02); //00000010 -> Canal 3
	NRF24_WriteRegisterByte(nrf, NRF_REG_RF_SETUP, 0x0e); //00001110 -> LNA desligado, 0dBm, 2MBs
	NRF24_WriteRegisterByte(nrf, NRF_REG_STATUS, 0x70); // clear flags

	uint8_t address[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t address2[5] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };
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
}

NRF_Status NRF24_WritePayload(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	return NRF24_SendWriteCommand(nrf, NRF_CMD_W_TX_PAYLOAD, payload, length);
}

NRF_Status NRF24_WritePayloadNoAck(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	return NRF24_SendWriteCommand(nrf, NRF_CMD_W_TX_PAYLOAD_NO_ACK, payload,
			length);
}

NRF_Status NRF24_WriteAckPayload(NRF24 *nrf, uint8_t pipe, uint8_t *payload,
		uint8_t length) {
	return NRF24_SendWriteCommand(nrf, NRF_CMD_W_ACK_PAYLOAD | pipe, payload,
			length);
}

NRF_Status NRF24_ReadPayload(NRF24 *nrf, uint8_t *read, uint8_t length) {
	return NRF24_SendReadCommand(nrf, NRF_CMD_R_RX_PAYLOAD, read, length);
}

NRF_Status NRF24_Transmit(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	NRF_Status ret = NRF_OK;
	ret = NRF24_WritePayload(nrf, payload, length);
	if (ret != NRF_OK) {
		return ret;
	}

	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
	wait(10);
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();

	return ret;
}

void NRF24_ReTransmit(NRF24 *nrf) {
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();
	wait(10);
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_RESET); //ce_reset();
}

NRF_Status NRF24_TransmitAndWait(NRF24 *nrf, uint8_t *payload, uint8_t length) {
	NRF_Status ret = NRF_OK;

	ret = NRF24_WritePayload(nrf, payload, length);
	if (ret != NRF_OK) {

		return ret;
	}

	// Transmit
	HAL_GPIO_WritePin(nrf->cePinBank, nrf->cePin, GPIO_PIN_SET); //ce_set();

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

void Tx_mode(NRF24 *nrf, uint8_t Adress[5]) {
	if (NRF24_Init(nrf, nrf->spiHandle, nrf->csPin, nrf->csPinBank, nrf->cePin, nrf->cePinBank) != NRF_OK) {
		Error_Handler();
	}
	NRF24_Reset(nrf);
	NRF24_WriteRegister(nrf, NRF_REG_TX_ADDR, Adress, 5);
}

void Rx_mode(NRF24 *nrf, uint8_t Adress[5], uint8_t dataSize) {
	if (NRF24_Init(nrf, nrf->spiHandle, nrf->csPin, nrf->csPinBank, nrf->cePin,
			nrf->cePinBank) != NRF_OK) {
		Error_Handler();
	}
	NRF24_Reset(nrf);
	NRF24_WriteRegister(nrf, NRF_REG_RX_ADDR_P0, Adress, 5);
	NRF24_WriteRegisterByte(nrf, NRF_REG_RX_PW_P0, dataSize);
	NRF24_EnterMode(nrf, NRF_MODE_RX);
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

/*
void changeChannel(NRF24 *nrf, uint8_t n) {
	NRF_EnterMode(nrf, NRF_MODE_STANDBY1);
	if (n == 0) {
		NRF_WriteRegisterByte(nrf, NRF_REG_RF_CH, 0x02); //Canal 3
	}
	if (n == 1) {
		NRF_WriteRegisterByte(nrf, NRF_REG_RF_CH, 0x03); //Canal 4
	}
	if (n == 2) {
		NRF_WriteRegisterByte(nrf, NRF_REG_RF_CH, 0x04); //Canal 5
	}
	NRF_EnterMode(nrf, NRF_MODE_TX);
}
*/

//NRF_Status NRF24_SendWriteCommand(NRF24 *nrf, uint8_t cmd, uint8_t *write, uint8_t length) {
/*
int NRF24_CurrentMode(NRF24 *nrf){
	 uint8_t config = NRF24_ReadRegisterByte(nrf, NRF_REG_CONFIG);

	 if (!(config & (1 << NRF_CONFIG_PWR_UP)))
		 return NRF_MODE_POWERDOWN;

	 if ((config & (1 << NRF_CONFIG_PRIM_RX)))
		 return NRF_MODE_RX;

	 if (NRF24_ReadRegisterByte(nrf, NRF_REG_FIFO_STATUS) & (1 << NRF_FIFO_STATUS_TX_EMPTY))
		 return NRF_MODE_STANDBY1;

	 return NRF_MODE_TX;
}
*/
NRF_Status NRF24_VerifySPI(NRF24 *nrf){
	uint8_t status = NRF24_ReadRegisterByte(nrf, NRF_REG_STATUS);

	return status;
}



void NRF24_PrintStatus(NRF24* nrf) {
    uint8_t status = NRF24_ReadRegisterByte(nrf, NRF_REG_STATUS);
    printf("Status Register: 0x%02X\n", status);
    printf("RX_DR: %d\n", (status >> 6) & 0x01);
    printf("TX_DS: %d\n", (status >> 5) & 0x01);
    printf("MAX_RT: %d\n", (status >> 4) & 0x01);
    printf("TX_FULL: %d\n", (status >> 0) & 0x01);
}

void NRF24_PrintFIFOStatus(NRF24 *nrf){
	uint8_t fifo_status = NRF24_ReadRegisterByte(nrf, NRF_REG_FIFO_STATUS);
	printf("FIFO Status Register: 0x%02X\n", fifo_status);
	printf("TX_REUSE: %d\n", (fifo_status >> 6) & 0x01);
	printf("TX_FULL: %d\n", (fifo_status >> 5) & 0x01);
	printf("TX_EMPTY: %d\n", (fifo_status >> 4) & 0x01);
	printf("RX_FULL: %d\n", (fifo_status >> 1) & 0x01);
	printf("RX_EMPTY: %d\n", (fifo_status >> 0) & 0x01);
}

void NRF24_PrintConfig(NRF24 *nrf){
	uint8_t config = NRF24_ReadRegisterByte(nrf, NRF_REG_CONFIG);
	printf("Config Register: 0x%02X\n", config);
    printf("MASK_RX_DR: %d\n", (config >> 6) & 0x01);
    printf("MASK_TX_DS: %d\n", (config >> 5) & 0x01);
    printf("MASK_MAX_RT: %d\n", (config >> 4) & 0x01);
    printf("EN_CRC: %d\n", (config >> 3) & 0x01);
    printf("CRCO: %d\n", (config >> 2) & 0x01);
    printf("PWR_UP: %d\n", (config >> 1) & 0x01);
    printf("PRIM_RX: %d\n", (config >> 0) & 0x01);
}
