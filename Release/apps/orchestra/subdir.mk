################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/orchestra/orchestra-rule-default-common.c \
../apps/orchestra/orchestra-rule-eb-per-time-source.c \
../apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-ns.c \
../apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-storing.c \
../apps/orchestra/orchestra.c 

OBJS += \
./apps/orchestra/orchestra-rule-default-common.o \
./apps/orchestra/orchestra-rule-eb-per-time-source.o \
./apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-ns.o \
./apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-storing.o \
./apps/orchestra/orchestra.o 

C_DEPS += \
./apps/orchestra/orchestra-rule-default-common.d \
./apps/orchestra/orchestra-rule-eb-per-time-source.d \
./apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-ns.d \
./apps/orchestra/orchestra-rule-unicast-per-neighbor-rpl-storing.d \
./apps/orchestra/orchestra.d 


# Each subdirectory must supply rules for building sources it contributes
apps/orchestra/%.o: ../apps/orchestra/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


