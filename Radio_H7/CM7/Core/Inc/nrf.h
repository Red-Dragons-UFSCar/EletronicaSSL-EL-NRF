/*
 * nrf.h
 *
 *  Created on: Aug 15, 2025
 *      Author: udavi
 */

#ifndef INC_NRF_H_
#define INC_NRF_H_

#include "stm32h7xx_hal.h"

/**
 * @name Commands
 * @anchor commands
 * @anchor command
 *
 * Comandos do NRF24L01
 *
 * Tabela 19 na página 48 do datasheet
 *
**/
#define NRF_CMD_R_REGISTER                0x00
#define NRF_CMD_W_REGISTER                0x20
#define NRF_CMD_R_RX_PAYLOAD              0x61
#define NRF_CMD_W_TX_PAYLOAD              0xA0
#define NRF_CMD_FLUSH_TX                  0xE1
#define NRF_CMD_FLUSH_RX                  0xE2
#define NRF_CMD_REUSE_TX_PL               0xE3
#define NRF_CMD_R_RX_PL_WID               0x60
#define NRF_CMD_W_ACK_PAYLOAD             0xA8
#define NRF_CMD_W_TX_PAYLOAD_NO_ACK       0xB0
#define NRF_CMD_NOP                       0xFF


/**
 * @name Registers
 * @anchor registers
 * @anchor register
 *
 * Registradores do NRF24
 *
 * Tabela 27 dapágina 57 do datasheet
 */
//!@{
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_OBSERVE_TX  0x08
#define NRF_REG_RPD         0x09
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_RX_ADDR_P1  0x0B
#define NRF_REG_RX_ADDR_P2  0x0C
#define NRF_REG_RX_ADDR_P3  0x0D
#define NRF_REG_RX_ADDR_P4  0x0E
#define NRF_REG_RX_ADDR_P5  0x0F
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_RX_PW_P1    0x12
#define NRF_REG_RX_PW_P2    0x13
#define NRF_REG_RX_PW_P3    0x14
#define NRF_REG_RX_PW_P4    0x15
#define NRF_REG_RX_PW_P5    0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD       0x1C
#define NRF_REG_FEATURE     0x1D
//!@}

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

#define NRF_SPI_TIMEOUT_TIME 10

typedef enum
{
  NRF_OK              = 0x00, //!< All went well
  NRF_SPI_ERROR       = 0x01, //!< Unspecified SPI error
  NRF_SPI_BUSY        = 0x02, //!< SPI device busy
  NRF_SPI_TIMEOUT     = 0x03, //!< SPI communication timed out
  NRF_ERROR           = 0x04, //!< Unspecified general error
  NRF_MAX_RT          = 0x05, //!< Max retries on packet transmission
  NRF_BAD_TRANSITION  = 0x06  //!< When an unallowed mode transition is tried
} NRF_Status;

typedef enum
{
  NRF_Mode_Powerdown = 0x00,
  NRF_Mode_Standby = 0x01,
  NRF_Mode_RxMode = 0x02,
  NRF_Mode_TxMode = 0x03
} NRF_Mode;


typedef struct
{
	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *csPinBank;
	GPIO_TypeDef *cePinBank;
	uint16_t csPin;
	uint16_t cePin;
	/* Modo */
	NRF_Mode Mode;
	/* DMA */
	int Mensagem[5];

} NRF24;

void wait(uint64_t us);

void Set_NRF24(NRF24 *nrf, SPI_HandleTypeDef *handle, GPIO_TypeDef *PortCS, uint16_t PinCs, GPIO_TypeDef *PortCE, uint16_t PinCE);

NRF_Status NRF24_SendCommand(NRF24 *nrf, uint8_t cmd);

NRF_Status NRF24_SendWriteCommand(NRF24 *nrf, uint8_t cmd , uint8_t *write, uint8_t length);

NRF_Status NRF24_SendReadCommand(NRF24 *nrf, uint8_t cmd, uint8_t *read, uint8_t length);

NRF_Status NRF24_WriteRegister(NRF24 *nrf,uint8_t reg, uint8_t *write, uint8_t Length);

NRF_Status NRF24_WriteRegisterByte(NRF24 *nrf,uint8_t reg, uint8_t byte);

NRF_Status NRF24_ReadRegister(NRF24 *nrf,uint8_t reg, uint8_t *read, uint8_t Length);

uint8_t NRF24_ReadRegisterByte(NRF24 *nrf, uint8_t byte);

NRF_Status NRF24_SetRegisterBit(NRF24 *nrf, uint8_t reg, uint8_t bit);

NRF_Status NRF24_ResetRegisterBit(NRF24 *nrf, uint8_t reg, uint8_t bit);

uint8_t NRF_ReadStatus(NRF24 *nrf);

NRF_Status NRF24_EnterMode(NRF24 *nrf, uint8_t mode);

NRF_Status NRF24_Config(NRF24 *nrf);

void Tx_mode(NRF24 *nrf, uint8_t Address[5]);

void Rx_mode(NRF24 *nrf, uint8_t Address[5], uint8_t dataSize);

NRF_Status NRF24_WritePayload(NRF24 *nrf, uint8_t *payload, uint8_t length);

NRF_Status NRF24_TransmitAndWait(NRF24 *nrf, uint8_t *payload, uint8_t length);

NRF_Status NRF24_Transmit(NRF24 *nrf, uint8_t *payload, uint8_t length);

NRF_Status NRF24_ReadPayload(NRF24 *nrf, uint8_t *read, uint8_t length);

NRF_Status ReceiveData(NRF24 *nrf, uint8_t *data, uint32_t len);

/* DMA */

NRF_Status NRF24_SendReadCommand_DMA(NRF24 *nrf, uint8_t cmd, uint8_t *read, uint8_t length);

NRF_Status NRF24_ReadPayload_DMA(NRF24 *nrf, uint8_t *read, uint8_t length);

NRF_Status ReceiveData_DMA (NRF24 *nrf);

NRF_Status NRF24_SetRegisterBit_DMA(NRF24 *nrf, uint8_t reg, uint8_t bit);

NRF_Status NRF24_ReadRegister_DMA(NRF24 *nrf, uint8_t reg, uint8_t *read, uint8_t Length);

NRF_Status NRF24_WriteRegister_DMA(NRF24 *nrf, uint8_t reg, uint8_t *write, uint8_t Length);

NRF_Status NRF24_SendWriteCommand_DMA(NRF24 *nrf, uint8_t cmd, uint8_t *write, uint8_t length);
#endif /* INC_NRF_H_ */
