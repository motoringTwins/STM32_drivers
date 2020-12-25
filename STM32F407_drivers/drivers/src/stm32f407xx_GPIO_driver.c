/*
 * stm32f407xx_GPIO_driver.c
 *
 *  Created on: Dec 18, 2020
 *      Author: motoringtwins
 */

#include "stm32f407xx_GPIO_driver.h"

/*
 * GPIO Peripheral clockCfg API
 */
void GPIO_PheripheralClockCfg(uint8_t PORT, uint8_t EnOrDis)
{
	if(ENABLE == EnOrDis)
	{
		GPIOPORTx_CLK_EN(PORT);
	}
	else
	{
		GPIOPORTx_CLK_DI(PORT);
	}
}


/*
 * GPIO Peripheral Initialization and De-initialization
 */
void GPIO_init(GPIO_handle_tst * handle_pst)
{
	uint32_t tempRegValue = 0U;

	/*Clock enable*/
	//GPIO_PheripheralClockCfg(handle_pst->GPIOx_pst, ENABLE);
	/*Configure mode of GPIO pin*/
	if(handle_pst->GPIO_pinCfg_st.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		tempRegValue = (handle_pst->GPIO_pinCfg_st.GPIO_PinMode << (2 * handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
		handle_pst->GPIOx_pst->MODER &= ~(0x3 << (2 * handle_pst->GPIO_pinCfg_st.GPIO_pinNo));  //clearing
		handle_pst->GPIOx_pst->MODER |= tempRegValue;										//setting
		tempRegValue = 0U;
	}
	else
	{
		//Interrupt mode
		if(handle_pst->GPIO_pinCfg_st.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			/*Configure for edge trigger falling edge*/
			EXTI->FTSR |= (1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
			EXTI->RTSR &= ~(1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);

		}
		else if (handle_pst->GPIO_pinCfg_st.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			/*Configure for edge trigger raising edge*/
			EXTI->RTSR |= (1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
			EXTI->FTSR &= ~(1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
		}
		else
		{
			/*Configure for edge trigger falling raising edge*/
			EXTI->FTSR |= (1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
			EXTI->RTSR |= (1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
		}

		SYSCFG_CLK_EN();
		uint32_t tempReg1 = handle_pst->GPIO_pinCfg_st.GPIO_pinNo/4;
		uint32_t tempReg2 = handle_pst->GPIO_pinCfg_st.GPIO_pinNo%4;
		uint8_t portNumber = GPIO_GETPORTNUMBER(handle_pst->GPIOx_pst);
		SYSCFG->EXTICR[tempReg1] |= (portNumber << (4 * tempReg2));
		/*Enable Interrupt delivery*/
		EXTI->IMR |= (1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
	}

	/*Configure Output type*/

	tempRegValue = (handle_pst->GPIO_pinCfg_st.GPIO_PinOutType << (handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
	handle_pst->GPIOx_pst->OTYPER &= ~(0x1 << handle_pst->GPIO_pinCfg_st.GPIO_pinNo);
	handle_pst->GPIOx_pst->OTYPER |= tempRegValue;
	tempRegValue = 0U;

	/*Configure the speed of GPIO pin*/

	tempRegValue = (handle_pst->GPIO_pinCfg_st.GPIO_PinOutSpeed << (2 *handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
	handle_pst->GPIOx_pst->OSPEEDR &= ~(0x3 << (2 * handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
	handle_pst->GPIOx_pst->OSPEEDR |= tempRegValue;
	tempRegValue = 0U;

	/*Configure pull up pull down*/
	tempRegValue = (handle_pst->GPIO_pinCfg_st.GPIO_PinpupdControl << (2 *handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
	handle_pst->GPIOx_pst->PUPDR &= ~(0x3 << (2 * handle_pst->GPIO_pinCfg_st.GPIO_pinNo));
	handle_pst->GPIOx_pst->PUPDR |= tempRegValue;
	tempRegValue = 0U;

	/*Configure Alternate function mode*/
	if(handle_pst->GPIO_pinCfg_st.GPIO_PinMode == GPIO_MODE_ALT_FNC)
	{
		uint32_t temp1 = (handle_pst->GPIO_pinCfg_st.GPIO_pinNo)/8;
		uint32_t temp2 = (handle_pst->GPIO_pinCfg_st.GPIO_pinNo)%8;

		handle_pst->GPIOx_pst->AFR[temp1] &= ~ (0xF << (4* temp2));
		handle_pst->GPIOx_pst->AFR[temp1] |= handle_pst->GPIO_pinCfg_st.GPIO_PinAltFncMode << (4* temp2);
	}
}
void GPIO_deInit(uint8_t PORT)
{
	/*Reset from AHB1 Pheripheral */
	RCCregdef_tst * rcc_base = RCC;

	/*Set and clear*/
	rcc_base->AHB1RSTR |= 1<<PORT;
	rcc_base->AHB1RSTR &= ~(1<<PORT);

}


/*
 * GPIO Peripheral Read/Write API
 */
uint8_t GPIO_ReadInputPin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo)
{
	uint8_t value = 0U;
	value = (uint8_t)((GPIOx_pst->IDR >> pinNo) & 0x1);
	return value;
}
uint16_t GPIO_ReadInputPort(GPIOregdef_tst *GPIOx_pst)
{
	uint16_t value = 0U;
	value = (uint16_t)(GPIOx_pst->IDR);
	return value;
}
void GPIO_WriteOutputPin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		GPIOx_pst->ODR |= (1<<pinNo);
	}
	else
	{
		GPIOx_pst->ODR &= ~(1<<pinNo);
	}

}
void GPIO_WriteOutputPort(GPIOregdef_tst *GPIOx_pst, uint16_t value)
{
	GPIOx_pst->ODR = value;
}
void GPIO_TogglePin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo)
{
	GPIOx_pst->ODR ^= (1<<pinNo);

}


/*
 * GPIO Peripheral IRQ and ISR Handler
 */
void GPIO_IrqCfg(uint8_t IrqNo, uint8_t EnorDis)
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

void GPIO_IrqPrioCfg(uint8_t IrqNo, uint8_t IrqPrio)
{
	/*Find iprx register and its position*/
	uint8_t iprx = IrqNo / 4;
	uint8_t iprx_section = IrqNo % 4;

	*(NVIV_IPR0_BASEADDR + iprx) = (IrqPrio << ((8*iprx_section)+4));
}
void GPIO_IrqHandler(uint8_t pinNo)
{
	/*Clear the EXTI Pending register*/
	if (EXTI->PR & (1<<pinNo))
	{
		EXTI->PR |= (1 << pinNo);
	}

}
