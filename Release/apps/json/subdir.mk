################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/json/jsonparse.c \
../apps/json/jsontree.c 

OBJS += \
./apps/json/jsonparse.o \
./apps/json/jsontree.o 

C_DEPS += \
./apps/json/jsonparse.d \
./apps/json/jsontree.d 


# Each subdirectory must supply rules for building sources it contributes
apps/json/%.o: ../apps/json/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


