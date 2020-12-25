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
#include "string.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(void)
{
	for(uint32_t i = 0; i<500000/4; i++);
}

int main(void)
{
	GPIO_handle_tst GPIO_Led, GPIO_Button;
	memset(&GPIO_Led,0,sizeof(GPIO_Led));
	memset(&GPIO_Button,0,sizeof(GPIO_Button));


	GPIO_Led.GPIOx_pst = GPIOD;
	GPIO_Led.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN12;
	GPIO_Led.GPIO_pinCfg_st.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_pinCfg_st.GPIO_PinOutSpeed = GPIO_OUTSPEED_HIGH;
	GPIO_Led.GPIO_pinCfg_st.GPIO_PinOutType = GPIO_OTYPE_PUSHPULL;
	GPIO_Led.GPIO_pinCfg_st.GPIO_PinpupdControl = GPIO_PUPD_NONE;

	GPIO_Button.GPIOx_pst = GPIOA;
	GPIO_Button.GPIO_pinCfg_st.GPIO_pinNo = GPIO_PIN0;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinOutSpeed = GPIO_OUTSPEED_HIGH;
	GPIO_Button.GPIO_pinCfg_st.GPIO_PinpupdControl = GPIO_PUPD_NONE;

	GPIO_PheripheralClockCfg(GPIO_PORTD, ENABLE);
	GPIO_PheripheralClockCfg(GPIO_PORTA, ENABLE);

	GPIO_init(&GPIO_Led);
	GPIO_init(&GPIO_Button);

	/*Irq config*/
	GPIO_IrqCfg(IRQ_NO_EXTI0, ENABLE);
	GPIO_IrqPrioCfg(IRQ_NO_EXTI0, 15);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IrqHandler(GPIO_PIN0);
	GPIO_TogglePin(GPIOD, GPIO_PIN12);
}
