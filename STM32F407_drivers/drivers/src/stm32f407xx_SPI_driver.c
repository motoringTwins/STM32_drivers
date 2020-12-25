/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Dec 20, 2020
 *      Author: motoringtwins
 */
#include "stm32f407xx_SPI_driver.h"

static void SPI_TXEIrqHandler(SPI_handle_tst* handle_pst)
{
	//Check DFF
	if(handle_pst->SPIx_pst->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit mode
		handle_pst->SPIx_pst->DR = *((uint16_t*)handle_pst->TxBuff);
		handle_pst->TxLen--;
		handle_pst->TxLen--;
		(uint16_t*)((handle_pst->TxBuff)++);
	}
	else
	{
		//8bit mode
		handle_pst->SPIx_pst->DR = *(handle_pst->TxBuff);
		handle_pst->TxLen--;
		handle_pst->TxBuff++;
	}
	if(handle_pst->TxLen >0)
	{
		//Close SPI TX
		//clear TXEIE bit
		SPI_CloseTransmission(handle_pst);

		//SPI indication for completion
		SPI_ApplTransferCmplCallback(handle_pst, SPI_EVENT_TX_CMP);
	}
}

static void SPI_RXNEIrqHandler(SPI_handle_tst* handle_pst)
{
	//Check DFF
	if(handle_pst->SPIx_pst->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit mode
		*((uint16_t*)handle_pst->RxBuff) = (uint16_t) handle_pst->SPIx_pst->DR;
		handle_pst->RxLen--;
		handle_pst->RxLen--;
		(uint16_t*)((handle_pst->RxBuff)++);
	}
	else
	{
		//8bit mode
		*(handle_pst->RxBuff) = (uint8_t) handle_pst->SPIx_pst->DR;
		handle_pst->RxLen--;
		handle_pst->RxBuff++;
	}
	if(handle_pst->RxLen >0)
	{
		//Close SPI RX
		//clear RXNEIE bit
		SPI_CloseReception(handle_pst);

		//SPI indication for completion
		SPI_ApplTransferCmplCallback(handle_pst, SPI_EVENT_RX_CMP);
	}
}

static void SPI_OVRErrIrqHandler(SPI_handle_tst* handle_pst)
{
	uint8_t temp;

	//clear OVR flag
	if(handle_pst->TxState != SPI_BUSY_IN_TX)
	{
		temp = handle_pst->SPIx_pst->DR;
		temp = handle_pst->SPIx_pst->SR;
	}
	handle_pst->SPIx_pst->CR2 &= ~(1 << SPI_SR_OVR);
	(void)temp;

	//Inform Application
	SPI_ApplTransferCmplCallback(handle_pst, SPI_EVENT_OVR_ERR);

}


/* Peripheral clock enable API*/
void SPI_PheripheralClockCfg(SPIregdef_tst* SPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if (SPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if (SPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if (SPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
		else if (SPIx == SPI3)
		{
			SPI4_CLK_EN();
		}
	}
	else
	{
		if (SPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if (SPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if (SPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
		else if (SPIx == SPI3)
		{
			SPI4_CLK_DI();
		}
	}

}

/* SPI Init and de-init*/

void SPI_init(SPI_handle_tst * handle_pst)
{
	uint32_t tempreg = 0;

	//Clock Enable
	SPI_PheripheralClockCfg(handle_pst->SPIx_pst, ENABLE);

	/* Configure CR1 register*/

	//1: Configure Device mode
	tempreg = handle_pst->SPI_Config_tst.SPI_DeviceMode << SPI_CR1_MSTR;

	//2: Configure Bus Config
	if(handle_pst->SPI_Config_tst.SPI_BusConfig == SPI_BUSCFG_FULLDUPLEX)
	{
		//CLear BiDi mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(handle_pst->SPI_Config_tst.SPI_BusConfig == SPI_BUSCFG_HALFDUPLEX)
	{
		//Set BiDi Mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(handle_pst->SPI_Config_tst.SPI_BusConfig == SPI_BUSCFG_SIMPLEX_RX)
	{
		//Clear BiDi mode
		//Enable RX only
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	else
	{
		//Clear BiDi mode
		//Disable RX only
		tempreg |= (1 << SPI_CR1_BIDIMODE);
		tempreg &= ~(1 << SPI_CR1_RXONLY);
	}

	//3: Configure Baud rate
	tempreg |= (handle_pst->SPI_Config_tst.SPI_BaudRate << SPI_CR1_BR);

	//4. Configure CPOL
	tempreg |= (handle_pst->SPI_Config_tst.SPI_CPOL << SPI_CR1_CPOL);

	//5. Configure CPHA
	tempreg |= (handle_pst->SPI_Config_tst.SPI_CPHA << SPI_CR1_CPHA);

	//6. Configure DFF
	tempreg |= (handle_pst->SPI_Config_tst.SPI_DFF << SPI_CR1_DFF);

	//7. Configure SSM and SSI
	tempreg |= (handle_pst->SPI_Config_tst.SPI_SSM << SPI_CR1_SSM);

	tempreg |= (handle_pst->SPI_Config_tst.SPI_SSI << SPI_CR1_SSI);

	handle_pst->SPIx_pst->CR1 = tempreg;

}


void SPI_deInit(SPIregdef_tst* SPIx)
{
	//SPE
	SPIx->CR1 |= 1<<SPI_CR1_SPE;
}

/*SPI send and recieve*/

void SPI_SendData(SPIregdef_tst* SPIx, uint8_t * dataBuff_pu8, uint32_t dataLen_u32)
{
	//Loop through complete buff length
	while(dataLen_u32 >0)
	{
		//Wait until Tx buf is empty
		while(!(SPIx->SR & (1 << SPI_SR_TXE)));

		//Check DFF
		if(SPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bit mode
			SPIx->DR = *((uint16_t*)dataBuff_pu8);
			dataLen_u32--;
			dataLen_u32--;
			(uint16_t*)(dataBuff_pu8++);
		}
		else
		{
			//8bit mode
			SPIx->DR = *dataBuff_pu8;
			dataLen_u32--;
			dataBuff_pu8++;
		}
	}

}

void SPI_RecieveData(SPIregdef_tst* SPIx, uint8_t * dataBuff_pu8, uint32_t dataLen_u32)
{
	//Loop through complete buff length
	while(dataLen_u32 >0)
	{
		//Wait until Rx buf is Full
		while(!(SPIx->SR & (1 << SPI_SR_RXNE)));

		//Check DFF
		if(SPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bit mode
			*((uint16_t*)dataBuff_pu8) = SPIx->DR;
			dataLen_u32--;
			dataLen_u32--;
			(uint16_t*)(dataBuff_pu8++);
		}
		else
		{
			//8bit mode
			*dataBuff_pu8 = SPIx->DR;
			dataLen_u32--;
			dataBuff_pu8++;
		}
	}
}

/*Non Blocking SPI transmit API*/
uint8_t SPI_SendDataIT(SPI_handle_tst* handle_pst, uint8_t * dataBuff_pu8, uint32_t dataLen_u32)
{
	uint8_t spi_state = handle_pst->TxState;
	if(spi_state != SPI_BUSY_IN_TX)
	{
		//1. Save Tx buffer and len to global section
		handle_pst->TxBuff = dataBuff_pu8;
		handle_pst->TxLen = dataLen_u32;

		//2. Mark SPI as busy to block SPI module until transmitted
		handle_pst->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXE Interrupt enable
		handle_pst->SPIx_pst->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return spi_state;

}


/*Non Blocking SPI recieve API*/
uint8_t SPI_RecieveDataIT(SPI_handle_tst* handle_pst, uint8_t * dataBuff_pu8, uint32_t dataLen_u32)
{
	uint8_t spi_state = handle_pst->RxState;
	if(spi_state != SPI_BUSY_IN_RX)
	{
		//1. Save Tx buffer and len to global section
		handle_pst->RxBuff = dataBuff_pu8;
		handle_pst->RxLen = dataLen_u32;

		//2. Mark SPI as busy to block SPI module until Recieved
		handle_pst->RxState = SPI_BUSY_IN_RX;

		//3. Enable RXNE Interrupt enable
		handle_pst->SPIx_pst->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return spi_state;
}

/*SPI Interrupt config*/
void SPI_IrqCfg(uint8_t IrqNo, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		if(IrqNo <= 31)
		{
			/*Configure ISER0 of NVIC*/
			*NVIC_ISER0_BASEADDR |= (1 << IrqNo);
		}
		else if(IrqNo <= 63)
		{
			/*Configure ISER1 of NVIC*/
			*NVIC_ISER1_BASEADDR |= (1 << (IrqNo%32));
		}
		else if(IrqNo <= 95)
		{
			/*Configure ISER2 of NVIC*/
			*NVIC_ISER2_BASEADDR |= (1 << (IrqNo%64));
		}
	}
	else
	{
		if(IrqNo <= 31)
		{
			/*Configure ICER0 of NVIC*/
			*NVIC_ICER0_BASEADDR |= (1 << IrqNo);
		}
		else if(IrqNo <= 63)
		{
			/*Configure ICER1 of NVIC*/
			*NVIC_ICER1_BASEADDR |= (1 << (IrqNo%32));
		}
		else if(IrqNo <= 95)
		{
			/*Configure ICER2 of NVIC*/
			*NVIC_ICER2_BASEADDR |= (1 << (IrqNo%64));
		}
	}
}

void SPI_IrqPrioCfg(uint8_t IrqNo, uint8_t IrqPrio)
{
	/*Find iprx register and its position*/
	uint8_t iprx = IrqNo / 4;
	uint8_t iprx_section = IrqNo % 4;

	*(NVIV_IPR0_BASEADDR + iprx) = (IrqPrio << ((8*iprx_section)+4));
}

void SPI_IrqHandler(SPI_handle_tst * handle_pst)
{
	uint8_t temp1, temp2;
	//Check for TXE and TXEIE
	temp1 = handle_pst->SPIx_pst->SR & (1 << SPI_SR_TXE);
	temp2 = handle_pst->SPIx_pst->CR2 & (1 << SPI_CR2_TXEIE);

	if((temp1 == 1) && (temp2 == 1))
	{
		SPI_TXEIrqHandler(handle_pst);
	}

	//Check for RXNE and RXNEIE
	temp1 = handle_pst->SPIx_pst->SR & (1 << SPI_SR_RXNE);
	temp2 = handle_pst->SPIx_pst->CR2 & (1 << SPI_CR2_RXNEIE);

	if((temp1 == 1) && (temp2 == 1))
	{
		SPI_RXNEIrqHandler(handle_pst);
	}

	//Check for OVR
	temp1 = handle_pst->SPIx_pst->SR & (1 << SPI_SR_OVR);
	temp2 = handle_pst->SPIx_pst->CR2 & (1 << SPI_CR2_ERRIE);

	if((temp1 == 1) && (temp2 == 1))
	{
		SPI_OVRErrIrqHandler(handle_pst);
	}
}

void SPI_PeripheralEnable(SPIregdef_tst* SPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		SPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		SPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

uint8_t SPI_GetBusyFlagStatus(SPIregdef_tst* SPIx)
{
	return (SPIx->SR & (1 << SPI_SR_BSY));
}

void SPI_SSOEControl(SPIregdef_tst* SPIx, uint8_t EnorDis)
{
	if(ENABLE == EnorDis)
	{
		SPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		SPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_ClearOVRFlag(SPIregdef_tst* SPIx)
{
	uint8_t temp;
	temp = SPIx->DR;
	temp = SPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_handle_tst* handle_pst)
{
	handle_pst->SPIx_pst->CR2 &= ~(1 << SPI_CR2_TXEIE);
	handle_pst->TxBuff = NULL;
	handle_pst->TxLen = 0;
	handle_pst->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_handle_tst* handle_pst)
{
	handle_pst->SPIx_pst->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	handle_pst->RxBuff = NULL;
	handle_pst->RxLen = 0;
	handle_pst->RxState = SPI_READY;
}

/*
 * Weak application API
 */
__attribute__((weak)) void SPI_ApplTransferCmplCallback(SPI_handle_tst *handle_pst, uint8_t SPI_event)
{

}

