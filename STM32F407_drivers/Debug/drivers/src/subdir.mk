################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407xx_GPIO_driver.c \
../drivers/src/stm32f407xx_SPI_driver.c 

OBJS += \
./drivers/src/stm32f407xx_GPIO_driver.o \
./drivers/src/stm32f407xx_SPI_driver.o 

C_DEPS += \
./drivers/src/stm32f407xx_GPIO_driver.d \
./drivers/src/stm32f407xx_SPI_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f407xx_GPIO_driver.o: ../drivers/src/stm32f407xx_GPIO_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"F:/STM32_project/STM32_drivers/STM32F407_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f407xx_GPIO_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/src/stm32f407xx_SPI_driver.o: ../drivers/src/stm32f407xx_SPI_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"F:/STM32_project/STM32_drivers/STM32F407_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f407xx_SPI_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

