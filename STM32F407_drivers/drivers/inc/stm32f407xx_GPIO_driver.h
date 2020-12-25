/*
 * stm32f407xx_GPIO_driver.h
 *
 *  Created on: Dec 18, 2020
 *      Author: motoringtwins
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#define GPIO_PIN_SET                   ENABLE
#define GPIO_PIN_RESET                 DISABLE

#define GPIO_IRQ_SET                   ENABLE
#define GPIO_IRQ_RESET                 DISABLE

/*
 * GPIO ports
 */
#define GPIO_PORTA          0U
#define GPIO_PORTB          1U
#define GPIO_PORTC          2U
#define GPIO_PORTD          3U
#define GPIO_PORTE          4U
#define GPIO_PORTF          5U
#define GPIO_PORTG          5U
#define GPIO_PORTH          6U
#define GPIO_PORTI          7U
#define GPIO_PORTJ          8U
#define GPIO_PORTK          9U

/*
 * GPIO Pin numbers
 */
#define GPIO_PIN0       0U
#define GPIO_PIN1       1U
#define GPIO_PIN2       2U
#define GPIO_PIN3       3U
#define GPIO_PIN4       4U
#define GPIO_PIN5       5U
#define GPIO_PIN6       6U
#define GPIO_PIN7       7U
#define GPIO_PIN8       8U
#define GPIO_PIN9       9U
#define GPIO_PIN10      10U
#define GPIO_PIN11      11U
#define GPIO_PIN12      12U
#define GPIO_PIN13      13U
#define GPIO_PIN14      14U
#define GPIO_PIN15      15U

/*
 * GPIO pin Mode macros
 */
#define GPIO_MODE_IN          0U
#define GPIO_MODE_OUT         1U
#define GPIO_MODE_ALT_FNC     2U
#define GPIO_MODE_ANALOG      3U
#define GPIO_MODE_IT_FT       4U
#define GPIO_MODE_IT_RT       5U
#define GPIO_MODE_IT_RFT      6U

/*
 * GPIO output Type macros
 */
#define GPIO_OTYPE_PUSHPULL    0U
#define GPIO_OTYPE_OPENDRN     1U

/*
 * GPIO output Speed macros
 */
#define GPIO_OUTSPEED_LOW           0U
#define GPIO_OUTSPEED_MED           1U
#define GPIO_OUTSPEED_HIGH          2U
#define GPIO_OUTSPEED_VERYHIGH      3U

/*
 * GPIO Pull up and Pull down macros
 */
#define GPIO_PUPD_NONE           0U
#define GPIO_PUPD_PU             1U
#define GPIO_PUPD_PD             2U

/* Get PORT number from Base address*/
#define GPIO_GETPORTNUMBER(X) (X == GPIOA)? GPIO_PORTA:\
							  (X == GPIOB)? GPIO_PORTB:\
						      (X == GPIOC)? GPIO_PORTC:\
						      (X == GPIOD)? GPIO_PORTD:\
						      (X == GPIOE)? GPIO_PORTE:\
						      (X == GPIOF)? GPIO_PORTF:\
						      (X == GPIOG)? GPIO_PORTG:\
						      (X == GPIOH)? GPIO_PORTH:\
						      (X == GPIOI)? GPIO_PORTI:\
						      (X == GPIOJ)? GPIO_PORTJ:\
						      (X == GPIOK)? GPIO_PORTK: 0\

/*
 * GPIO pin configuration structure type
 */
typedef struct
{
	uint8_t GPIO_pinNo;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOutSpeed;
	uint8_t GPIO_PinpupdControl;
	uint8_t GPIO_PinOutType;
	uint8_t GPIO_PinAltFncMode;
}GPIOpinCfg_tst;

/*
 * GPIO handler type for port pin
 */
typedef struct
{
	GPIOregdef_tst *GPIOx_pst;
	GPIOpinCfg_tst GPIO_pinCfg_st;
}GPIO_handle_tst;

/*
 * GPIO Peripheral clockCfg API
 */
void GPIO_PheripheralClockCfg(uint8_t PORT, uint8_t EnOrDis);


/*
 * GPIO Peripheral Initialization and De-initialization
 */
void GPIO_init(GPIO_handle_tst * handle_pst);
void GPIO_deInit(uint8_t PORT);


/*
 * GPIO Peripheral Read/Write API
 */
uint8_t GPIO_ReadInputPin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo);
uint16_t GPIO_ReadInputPort(GPIOregdef_tst *GPIOx_pst);
void GPIO_WriteOutputPin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo, uint8_t value);
void GPIO_WriteOutputPort(GPIOregdef_tst *GPIOx_pst, uint16_t value);
void GPIO_TogglePin(GPIOregdef_tst *GPIOx_pst, uint8_t pinNo);


/*
 * GPIO Peripheral IRQ and ISR Handler
 */
void GPIO_IrqCfg(uint8_t IrqNo, uint8_t EnorDis);
void GPIO_IrqPrioCfg(uint8_t IrqNo, uint8_t IrqPrio);
void GPIO_IrqHandler(uint8_t pinNo);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
