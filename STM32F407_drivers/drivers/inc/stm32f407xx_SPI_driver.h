/*
 * stm32f407xx_SPI_driver.h
 *
 *  Created on: Dec 20, 2020
 *      Author: motoringtwins
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * SPI Peripheral Bit positions
 */

/*
 * SPI states
 */
#define SPI_READY         1
#define SPI_BUSY_IN_RX    2
#define SPI_BUSY_IN_TX    3

/*
 * APPLICATION EVENTS
 */
#define SPI_EVENT_TX_CMP  1
#define SPI_EVENT_RX_CMP  2
#define SPI_EVENT_OVR_ERR 3
#define SPI_EVENT_CRC_ERR 4



// CR1 register
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_BR          3
#define SPI_CR1_MSTR        2
#define SPI_CR1_SPE         6
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_SSM         9
#define SPI_CR1_SSI         8
#define SPI_CR1_BIDIMODE    15
#define SPI_CR1_BIDIOE      14

// CR2 register Bit
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

//Status register Bit
#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_BSYFRE       8

/*
 * SPI config
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BaudRate;       /* Clock speed*/
	uint8_t SPI_BusConfig;      /*Full duplex, half duplex, Simplex*/
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_DFF;
	uint8_t SPI_SSM;            /*Slave Software management*/
	uint8_t SPI_SSI;            /*Slave Software management*/
}SPIConfig_tst;

/*
 * SPI handle structure
 */

typedef struct
{
	SPIregdef_tst* SPIx_pst;
	SPIConfig_tst SPI_Config_tst;
	uint8_t      *TxBuff;
	uint8_t      *RxBuff;
	uint8_t      TxLen;
	uint8_t      RxLen;
	uint8_t      TxState;
	uint8_t      RxState;
}SPI_handle_tst;

/*
 * Device mode
 */
#define SPI_DEVMODE_MASTER     1
#define SPI_DEVMODE_SLAVE      0

/*
 * SPI BusConfig
 */
#define SPI_BUSCFG_FULLDUPLEX    0
#define SPI_BUSCFG_HALFDUPLEX    1
#define SPI_BUSCFG_SIMPLEX_TX    2
#define SPI_BUSCFG_SIMPLEX_RX    3

/*
 * Baud rate control
 */
#define SPI_CLKSPD_DIV2       0
#define SPI_CLKSPD_DIV4       1
#define SPI_CLKSPD_DIV8       2
#define SPI_CLKSPD_DIV16      3
#define SPI_CLKSPD_DIV32      4
#define SPI_CLKSPD_DIV64      5
#define SPI_CLKSPD_DIV128     6
#define SPI_CLKSPD_DIV256     7

/*
 * SPI_DFF
 */
#define SPI_DFF_8BITS      0
#define SPI_DFF_16BITS     1

/*
 * SPI CPOL
 */
#define SPI_CPOL_HIGH      1
#define SPI_CPOL_LOW       0

/*
 * SPI CPHA
 */
#define SPI_CPHA_HIGH      1
#define SPI_CPHA_LOW       0

/*
 * SPI SSM
 */
#define SPI_SSM_EN      1
#define SPI_SSM_DIS     0

/*
 * SPI SSI
 */
#define SPI_SSI_EN      1
#define SPI_SSI_DIS     0

/* Peripheral clock enable API*/
void SPI_PheripheralClockCfg(SPIregdef_tst* SPIx, uint8_t EnOrDis);

/* SPI Init and de-init*/

void SPI_init(SPI_handle_tst * handle_pst);
void SPI_deInit(SPIregdef_tst* SPIx);

/*SPI send and recieve*/

void SPI_SendData(SPIregdef_tst* SPIx, uint8_t * dataBuff_pu8, uint32_t dataLen_u32);
void SPI_RecieveData(SPIregdef_tst* SPIx, uint8_t * dataBuff_pu8, uint32_t dataLen_u32);

uint8_t SPI_SendDataIT(SPI_handle_tst* handle_pst, uint8_t * dataBuff_pu8, uint32_t dataLen_u32);
uint8_t SPI_RecieveDataIT(SPI_handle_tst* handle_pst, uint8_t * dataBuff_pu8, uint32_t dataLen_u32);

/*SPI Interrupt config*/
void SPI_IrqCfg(uint8_t IrqNo, uint8_t EnorDis);
void SPI_IrqPrioCfg(uint8_t IrqNo, uint8_t IrqPrio);
void SPI_IrqHandler(SPI_handle_tst * handle_pst);

/* Other API's
 *
 */
void SPI_PeripheralEnable(SPIregdef_tst* SPIx, uint8_t EnorDis);
uint8_t SPI_GetBusyFlagStatus(SPIregdef_tst* SPIx);
void SPI_SSOEControl(SPIregdef_tst* SPIx, uint8_t EnorDis);
void SPI_ClearOVRFlag(SPIregdef_tst* SPIx);
void SPI_CloseTransmission(SPI_handle_tst* handle_pst);
void SPI_CloseReception(SPI_handle_tst* handle_pst);

/*
 * Application callback
 */
void SPI_ApplTransferCmplCallback(SPI_handle_tst *handle_pst, uint8_t SPI_event);


/*SPI Interrupt handler*/
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
