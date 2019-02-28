################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/calc/calc-dsc.c \
../apps/calc/calc.c 

OBJS += \
./apps/calc/calc-dsc.o \
./apps/calc/calc.o 

C_DEPS += \
./apps/calc/calc-dsc.d \
./apps/calc/calc.d 


# Each subdirectory must supply rules for building sources it contributes
apps/calc/%.o: ../apps/calc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


