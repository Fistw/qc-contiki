################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/irc/irc-dsc.c \
../apps/irc/irc.c \
../apps/irc/ircc-strings.c \
../apps/irc/ircc.c 

OBJS += \
./apps/irc/irc-dsc.o \
./apps/irc/irc.o \
./apps/irc/ircc-strings.o \
./apps/irc/ircc.o 

C_DEPS += \
./apps/irc/irc-dsc.d \
./apps/irc/irc.d \
./apps/irc/ircc-strings.d \
./apps/irc/ircc.d 


# Each subdirectory must supply rules for building sources it contributes
apps/irc/%.o: ../apps/irc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


