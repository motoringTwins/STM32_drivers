/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_GPIO_driver.h"
#include "stm32f407xx_SPI_driver.h"
#include "string.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(void)
{
	for(uint32_t i = 0; i<500000/2; i++);
}

void GPIO_ButtonCfg()
{
	GPIO_handle_tst GPIO_Button;

	GPIO_Button.GPIOx_pst = GPIOA;
	GPIO_Button.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN0;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinOutSpeed = GPIO_OUTSPEED_HIGH;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinpupdControl = GPIO_PUPD_NONE;

	GPIO_PheripheralClockCfg(GPIO_PORTA, ENABLE);

	GPIO_init(&GPIO_Button);
}

/*SPI2 Pin Cfg
 * PB12     NSS
 * PB13     SCLK
 * PB14     MISO
 * PB15     MOSI
 */
void SPI2_PinCfg(void)
{
	GPIO_handle_tst SPI2_Pinshandle;
	memset(&SPI2_Pinshandle,0,sizeof(SPI2_Pinshandle));
	SPI2_Pinshandle.GPIOx_pst = GPIOB;

	//Enable GPIO B Peripheral
	GPIO_PheripheralClockCfg(GPIO_PORTB, ENABLE);

	//SCLK
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN13;
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_PinMode = GPIO_MODE_ALT_FNC;
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_PinOutSpeed = GPIO_OUTSPEED_HIGH;
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_PinOutType = GPIO_OTYPE_PUSHPULL;
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_PinpupdControl = GPIO_PUPD_PU;
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_PinAltFncMode = 5;
	GPIO_init(&SPI2_Pinshandle);

	//MISO
//	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN14;
//	GPIO_init(&SPI2_Pinshandle);

	//MOSI
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN15;
	GPIO_init(&SPI2_Pinshandle);

	//NSS
	SPI2_Pinshandle.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN12;
	GPIO_init(&SPI2_Pinshandle);
}

/*
 * SPI Peripheral Initialization
 */
void SPI2_InitConfig(void)
{
	SPI_handle_tst SPI2_handle;

	SPI2_handle.SPIx_pst = SPI2;

	SPI2_handle.SPI_Config_tst.SPI_DeviceMode = SPI_DEVMODE_MASTER;
	SPI2_handle.SPI_Config_tst.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_handle.SPI_Config_tst.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPI_Config_tst.SPI_BaudRate = SPI_CLKSPD_DIV8;    //2Mz SCLK
	SPI2_handle.SPI_Config_tst.SPI_BusConfig = SPI_BUSCFG_FULLDUPLEX;
	SPI2_handle.SPI_Config_tst.SPI_DFF = SPI_DFF_8BITS;
	//NSS pin to VCC for single master mode
	SPI2_handle.SPI_Config_tst.SPI_SSM = SPI_SSM_DIS;   //software Slave management disable since NSS is connected

	SPI_init(&SPI2_handle);

}

int main(void)
{
	char user_data[] = "Welcome to motoringtwins YOutube Channel and STM32 Driver development";
	uint8_t len = strlen(user_data);

	//Configure Port A 0pin for Button
	GPIO_ButtonCfg();

	/*SPI config*/
	SPI2_PinCfg();
	SPI2_InitConfig();

	while(1)
	{
		if(GPIO_ReadInputPin(GPIOA, GPIO_PIN0))
		{
			delay();

			/*
			 * Make SSOE to 1 since NSS pin is controlled as per SPE
			 */
			SPI_SSOEControl(SPI2, ENABLE);

			/*Enable SPI2 module*/
			SPI_PeripheralEnable(SPI2, ENABLE);

			//Send Length Information
			SPI_SendData(SPI2, &len, 1);

			/*SPI2 send data*/
			SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

			//lets confirm SPI is not busy
			while(SPI_GetBusyFlagStatus(SPI2));

			//Disable the SPI2 peripheral
			SPI_PeripheralEnable(SPI2,DISABLE);
		}

	}

	return 0;
}

