################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/soft_tmr.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/terminal.c 

OBJS += \
./Src/main.o \
./Src/soft_tmr.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/terminal.o 

C_DEPS += \
./Src/main.d \
./Src/soft_tmr.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/terminal.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Inc" -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Drivers/CMSIS/Include" -I"C:/Users/karol/Documents/STM32Projects/STM32F103_KickstartApp/STM32F103_KickstartApp/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


