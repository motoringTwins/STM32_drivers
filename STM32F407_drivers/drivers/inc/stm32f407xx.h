/*
 * stm32f407xx.h
 *
 *  Created on: Dec 18, 2020
 *      Author: motoringtwins
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include "stdint.h"

#define NULL ((void *)0)

/*
 * NVIC registers
 */
#define NVIC_ISER0_BASEADDR      ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1_BASEADDR      ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2_BASEADDR      ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3_BASEADDR      ((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4_BASEADDR      ((volatile uint32_t*)0xE000E110)
#define NVIC_ISER5_BASEADDR      ((volatile uint32_t*)0xE000E114)
#define NVIC_ISER6_BASEADDR      ((volatile uint32_t*)0xE000E118)
#define NVIC_ISER7_BASEADDR      ((volatile uint32_t*)0xE000E11C)

#define NVIC_ICER0_BASEADDR      ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1_BASEADDR      ((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2_BASEADDR      ((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3_BASEADDR      ((volatile uint32_t*)0XE000E18C)
#define NVIC_ICER4_BASEADDR      ((volatile uint32_t*)0XE000E190)
#define NVIC_ICER5_BASEADDR      ((volatile uint32_t*)0XE000E194)
#define NVIC_ICER6_BASEADDR      ((volatile uint32_t*)0XE000E198)
#define NVIC_ICER7_BASEADDR      ((volatile uint32_t*)0XE000E19C)


#define NVIV_IPR0_BASEADDR       ((volatile uint32_t*)0xE000E400)



/*
 * Base address of memory sections
 */
#define FLASH_BASEADDR               0x08000000U       /*Base address of Flash memory*/
#define SRAM1_BASEADDR               0x02000000U       /*Base address of SRAM1 memory*/
#define SRAM                         FLASH_SRAM1ADDR
#define SRAM2_BASEADDR               0x2001C000U       /*Base address of SRAM2 memory*/


/*
 * Peripheral base address of AHBx and APBx
 */

#define APB1PERIPHERAL_BASEADDR      0x40000000U      /*Base address of APB1 Peripheral*/
#define APB2PERIPHERAL_BASEADDR      0x40010000U      /*Base address of APB2 Peripheral*/
#define AHB1PERIPHERAL_BASEADDR      0x40020000U      /*Base address of AHB1 Peripheral*/
#define AHB2PERIPHERAL_BASEADDR      0x50000000U      /*Base address of AHB2 Peripheral*/

/*
 * Base address of Peripherals hanging on AHB1 bus
 */

#define GPIOA_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x0000U)     /*GPIOA Base Address*/
#define GPIOB_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x0400U)     /*GPIOB Base Address*/
#define GPIOC_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x0800U)     /*GPIOC Base Address*/
#define GPIOD_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x0C00U)     /*GPIOD Base Address*/
#define GPIOE_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x1000U)     /*GPIOE Base Address*/
#define GPIOF_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x1400U)     /*GPIOF Base Address*/
#define GPIOG_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x1800U)     /*GPIOG Base Address*/
#define GPIOH_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x1C00U)     /*GPIOH Base Address*/
#define GPIOI_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x2000U)     /*GPIOI Base Address*/
#define GPIOJ_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x2400U)     /*GPIOJ Base Address*/
#define GPIOK_BASEADDR               (AHB1PERIPHERAL_BASEADDR + 0x2800U)     /*GPIOK Base Address*/


#define RCC_BASEADDR                 (AHB1PERIPHERAL_BASEADDR + 0x3800U)    /*RCC Base address*/

/*
 * Base address of Peripherals hanging on APB1 bus
 */

#define TIM2_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x0000U)     /*TIM2 Base Address*/
#define TIM3_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x0400U)     /*TIM3 Base Address*/
#define TIM4_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x0800U)     /*TIM4 Base Address*/
#define TIM5_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x0C00U)     /*TIM5 Base Address*/
#define TIM6_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x1000U)     /*TIM6 Base Address*/
#define TIM7_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x1400U)     /*TIM7 Base Address*/
#define TIM12_BASEADDR               (APB1PERIPHERAL_BASEADDR + 0x1800U)     /*TIM12 Base Address*/
#define TIM13_BASEADDR               (APB1PERIPHERAL_BASEADDR + 0x1C00U)     /*TIM13 Base Address*/
#define TIM14_BASEADDR               (APB1PERIPHERAL_BASEADDR + 0x2000U)     /*TIM14 Base Address*/

#define SPI2_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x3800U)     /*SPI2 Base Address*/
#define SPI3_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x3C00U)     /*SPI3 Base Address*/

#define USART2_BASEADDR              (APB1PERIPHERAL_BASEADDR + 0x4400U)     /*USART2 Base Address*/
#define USART3_BASEADDR              (APB1PERIPHERAL_BASEADDR + 0x4800U)     /*USART3 Base Address*/
#define UART4_BASEADDR               (APB1PERIPHERAL_BASEADDR + 0x4C00U)     /*UART4 Base Address*/
#define UART5_BASEADDR               (APB1PERIPHERAL_BASEADDR + 0x5000U)     /*UART5 Base Address*/

#define I2C1_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x5400U)     /*I2C1 Base Address*/
#define I2C2_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x5800U)     /*I2C2 Base Address*/
#define I2C3_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x5C00U)     /*I2C3 Base Address*/

#define CAN1_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x6400U)     /*CAN1 Base Address*/
#define CAN2_BASEADDR                (APB1PERIPHERAL_BASEADDR + 0x6800U)     /*CAN2 Base Address*/

/*
 * Base address of Peripherals hanging on APB2 bus
 * ToDO  Later
 */
#define EXTI_BASEADDR                (APB2PERIPHERAL_BASEADDR + 0x3C00)      /* EXTI*/
#define SYSCFG_BASEADDR              (APB2PERIPHERAL_BASEADDR + 0x3800)      /* SYSCFG*/
#define SPI1_BASEADDR                (APB2PERIPHERAL_BASEADDR + 0x3000)      /*SPI1*/
#define SPI4_BASEADDR                (APB2PERIPHERAL_BASEADDR + 0x3400)      /*SPI4*/
#define SPI5_BASEADDR                (APB2PERIPHERAL_BASEADDR + 0x5000)      /*SPI5*/
#define SPI6_BASEADDR                (APB2PERIPHERAL_BASEADDR + 0x5400)      /*SPI6*/

/*
 * GPIOx Peripheral register structure
 */

typedef struct
{
	volatile uint32_t MODER;       /*GPIO port mode register                Address offset:0x00*/
	volatile uint32_t OTYPER;      /*GPIO port output type register         Address offset:0x04*/
	volatile uint32_t OSPEEDR;     /*GPIO port output speed register        Address offset:0x08*/
	volatile uint32_t PUPDR;       /*GPIO port pull-up/pull-down register   Address offset:0x0C*/
	volatile uint32_t IDR;         /*GPIO port input data register          Address offset:0x10*/
	volatile uint32_t ODR;         /*GPIO port output data register         Address offset:0x14*/
	volatile uint32_t BSRR;        /*GPIO port bit set/reset register       Address offset:0x18*/
	volatile uint32_t LCKR;        /*GPIO port configuration lock register  Address offset:0x1C*/
	volatile uint32_t AFR[2];      /*GPIO alternate function low high register Address offset: 0x20 0x24*/
}GPIOregdef_tst;

/*
 * Peripheral definitions of GPIO typecasted
 */

#define GPIOA          ((GPIOregdef_tst*)GPIOA_BASEADDR)
#define GPIOB          ((GPIOregdef_tst*)GPIOB_BASEADDR)
#define GPIOC          ((GPIOregdef_tst*)GPIOC_BASEADDR)
#define GPIOD          ((GPIOregdef_tst*)GPIOD_BASEADDR)
#define GPIOE          ((GPIOregdef_tst*)GPIOE_BASEADDR)
#define GPIOF          ((GPIOregdef_tst*)GPIOF_BASEADDR)
#define GPIOG          ((GPIOregdef_tst*)GPIOG_BASEADDR)
#define GPIOH          ((GPIOregdef_tst*)GPIOH_BASEADDR)
#define GPIOI          ((GPIOregdef_tst*)GPIOI_BASEADDR)
#define GPIOJ          ((GPIOregdef_tst*)GPIOJ_BASEADDR)
#define GPIOK          ((GPIOregdef_tst*)GPIOK_BASEADDR)

/*
 * RCC Peripheral register structure
 */

typedef struct
{
	volatile uint32_t CR;               /*RCC clock control register                 offset 0x00*/
	volatile uint32_t PLLCFGR;          /*RCC PLL configuration register             offset 0x04*/
	volatile uint32_t CFGR;             /*RCC clock configuration register           offset 0x08*/
	volatile uint32_t CIR;              /*RCC clock interrupt register               offset 0x0C*/
	volatile uint32_t AHB1RSTR;         /*RCC AHB1 peripheral reset register         offset 0x10*/
	volatile uint32_t AHB2RSTR;         /*RCC AHB2 peripheral reset register         offset 0x14*/
	volatile uint32_t AHB3RSTR;         /*RCC AHB3 peripheral reset register         offset 0x18*/
	uint32_t          RESERVED0;
	volatile uint32_t APB1RSTR;         /*RCC APB1 peripheral reset register         offset 0x20*/
	volatile uint32_t APB2RSTR;         /*RCC APB2 peripheral reset register         offset 0x24*/
	uint32_t          RESERVED1[2];
	volatile uint32_t AHB1ENR;			/*RCC AHB1 peripheral clock enable register  offset 0x30*/
	volatile uint32_t AHB2ENR;			/*RCC AHB2 peripheral clock enable register  offset 0x34*/
	volatile uint32_t AHB3ENR;			/*RCC AHB3 peripheral clock enable register  offset 0x38*/
	uint32_t          RESERVED2;
	volatile uint32_t APB1ENR;			/*RCC APB1 peripheral clock enable register  offset 0x40*/
	volatile uint32_t APB2ENR;			/*RCC APB2 peripheral clock enable register  offset 0x44*/
	uint32_t          RESERVED3[2];
	volatile uint32_t AHB1LPENR;		/*RCC AHB1 peripheral clock enable in low power mode register           offset 0x50*/
	volatile uint32_t AHB2LPENR;		/*RCC AHB2 peripheral clock enable in low power mode register           offset 0x54*/
	volatile uint32_t AHB3LPENR;        /*RCC AHB3 peripheral clock enable in low power mode register           offset 0x58*/
	uint32_t          RESERVED4;
	volatile uint32_t APB1LPENR;        /*RCC APB1 peripheral clock enable in low power mode register           offset 0x60*/
	volatile uint32_t APB2LPENR;        /*RCC APB2 peripheral clock enable in low power mode register           offset 0x64*/
	uint32_t          RESERVED5[2];
	volatile uint32_t BDCR;             /*RCC Backup domain control register         offset 0x70*/
	volatile uint32_t CSR;              /*RCC clock control & status register        offset 0x74*/
	uint32_t          RESERVED6[2];
	volatile uint32_t SSCGR;            /*RCC spread spectrum clock generation register          offset 0x80*/
	volatile uint32_t PLLI2SCFGR;       /*RCC PLLI2S configuration register           offset 0x84*/
}RCCregdef_tst;

/*
 * Peripheral definitions of RCC typecasted
 */
#define RCC          ((RCCregdef_tst*)RCC_BASEADDR)

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTIregdef_tst;

/*
 * Peripheral definitions of EXTI typecasted
 */
#define EXTI          ((EXTIregdef_tst*)EXTI_BASEADDR)

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t          RESERVED1[2];
	volatile uint32_t CMPCR;
	uint32_t          RESERVED2[2];
	volatile uint32_t CFGR;
}SysCfgRegdef_tst;

#define SYSCFG          ((SysCfgRegdef_tst*)SYSCFG_BASEADDR)

/*
 * Peripheral clock enable disable
 */

/*
 * GPIOx clock Enable/Disable macros
 */

#define GPIOPORTx_CLK_EN(x)            ((RCC->AHB1ENR) |= (1<<x))

#define GPIOA_CLK_EN()            ((RCC->AHB1ENR) |= (1<<0))
#define GPIOB_CLK_EN()            ((RCC->AHB1ENR) |= (1<<1))
#define GPIOC_CLK_EN()            ((RCC->AHB1ENR) |= (1<<2))
#define GPIOD_CLK_EN()            ((RCC->AHB1ENR) |= (1<<3))
#define GPIOE_CLK_EN()            ((RCC->AHB1ENR) |= (1<<4))
#define GPIOF_CLK_EN()            ((RCC->AHB1ENR) |= (1<<5))
#define GPIOG_CLK_EN()            ((RCC->AHB1ENR) |= (1<<6))
#define GPIOH_CLK_EN()            ((RCC->AHB1ENR) |= (1<<7))
#define GPIOI_CLK_EN()            ((RCC->AHB1ENR) |= (1<<8))

#define GPIOPORTx_CLK_DI(x)       ((RCC->AHB1ENR) &= ~(1<<x))

#define GPIOA_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<0))
#define GPIOB_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<1))
#define GPIOC_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<2))
#define GPIOD_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<3))
#define GPIOE_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<4))
#define GPIOF_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<5))
#define GPIOG_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<6))
#define GPIOH_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<7))
#define GPIOI_CLK_DI()           ((RCC->AHB1ENR) &= ~(1<<8))

/*
 * IRQ(Interrupt Request) Number of STM32F407x MCU
 */
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

/*
 * SPI IRQ numbers
 */
#define IRQ_NO_SPI1   35
#define IRQ_NO_SPI2   36
#define IRQ_NO_SPI3   51

/*
 * I2C clock Enable/Disable macros
 */
#define I2C1_CLK_EN()            ((RCC->APB1ENR) |= (1<<21))
#define I2C2_CLK_EN()            ((RCC->APB1ENR) |= (1<<22))
#define I2C3_CLK_EN()            ((RCC->APB1ENR) |= (1<<23))

#define I2C1_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<21))
#define I2C2_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<22))
#define I2C3_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<23))


/*
 * SPI clock Enable/Disable macros
 */
#define SPI1_CLK_EN()            ((RCC->APB2ENR) |= (1<<12))
#define SPI2_CLK_EN()            ((RCC->APB1ENR) |= (1<<14))
#define SPI3_CLK_EN()            ((RCC->APB1ENR) |= (1<<15))
#define SPI4_CLK_EN()            ((RCC->APB2ENR) |= (1<<13))


#define SPI1_CLK_DI()            ((RCC->APB2ENR) &= ~(1<<12))
#define SPI2_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<14))
#define SPI3_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<15))
#define SPI4_CLK_DI()            ((RCC->APB2ENR) &= ~(1<<13))

/*
 * UART/USART clock Enable/Disable macros
 */
#define USART1_CLK_EN()            ((RCC->APB2ENR) |= (1<<4))
#define USART2_CLK_EN()            ((RCC->APB1ENR) |= (1<<17))
#define USART3_CLK_EN()            ((RCC->APB1ENR) |= (1<<18))
#define UART4_CLK_EN()             ((RCC->APB1ENR) |= (1<<19))
#define UART5_CLK_EN()             ((RCC->APB1ENR) |= (1<<20))
#define USART6_CLK_EN()            ((RCC->APB2ENR) |= (1<<5))



#define USART1_CLK_DI()            ((RCC->APB2ENR) &= ~(1<<4))
#define USART2_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<17))
#define USART3_CLK_DI()            ((RCC->APB1ENR) &= ~(1<<18))
#define UART4_CLK_DI()             ((RCC->APB1ENR) &= ~(1<<19))
#define UART5_CLK_DI()             ((RCC->APB1ENR) &= ~(1<<20))
#define USART6_CLK_DI()            ((RCC->APB2ENR) &= ~(1<<5))

/*
 * SYSCFG clock Enable/Disable macros
 */

#define SYSCFG_CLK_EN()            ((RCC->APB2ENR) |= (1<<14))

#define SYSCFG_CLK_DI()            ((RCC->APB2ENR) &= ~(1<<14))

/*
 * SPI register def
 */

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPIregdef_tst;

#define SPI1     (SPIregdef_tst*)(SPI1_BASEADDR)
#define SPI2     (SPIregdef_tst*)(SPI2_BASEADDR)
#define SPI3     (SPIregdef_tst*)(SPI3_BASEADDR)
#define SPI4     (SPIregdef_tst*)(SPI4_BASEADDR)

/*
 * Utility Macros
 */
#define ENABLE                     (uint8_t)0x01
#define DISABLE                    (uint8_t)0x00

#endif /* INC_STM32F407XX_H_ */
