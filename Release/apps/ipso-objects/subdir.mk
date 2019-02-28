################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/ipso-objects/ipso-button.c \
../apps/ipso-objects/ipso-leds-control.c \
../apps/ipso-objects/ipso-light-control.c \
../apps/ipso-objects/ipso-objects.c \
../apps/ipso-objects/ipso-temperature.c 

OBJS += \
./apps/ipso-objects/ipso-button.o \
./apps/ipso-objects/ipso-leds-control.o \
./apps/ipso-objects/ipso-light-control.o \
./apps/ipso-objects/ipso-objects.o \
./apps/ipso-objects/ipso-temperature.o 

C_DEPS += \
./apps/ipso-objects/ipso-button.d \
./apps/ipso-objects/ipso-leds-control.d \
./apps/ipso-objects/ipso-light-control.d \
./apps/ipso-objects/ipso-objects.d \
./apps/ipso-objects/ipso-temperature.d 


# Each subdirectory must supply rules for building sources it contributes
apps/ipso-objects/%.o: ../apps/ipso-objects/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


