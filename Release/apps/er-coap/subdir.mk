################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/er-coap/er-coap-block1.c \
../apps/er-coap/er-coap-engine.c \
../apps/er-coap/er-coap-observe-client.c \
../apps/er-coap/er-coap-observe.c \
../apps/er-coap/er-coap-res-well-known-core.c \
../apps/er-coap/er-coap-separate.c \
../apps/er-coap/er-coap-transactions.c \
../apps/er-coap/er-coap.c 

OBJS += \
./apps/er-coap/er-coap-block1.o \
./apps/er-coap/er-coap-engine.o \
./apps/er-coap/er-coap-observe-client.o \
./apps/er-coap/er-coap-observe.o \
./apps/er-coap/er-coap-res-well-known-core.o \
./apps/er-coap/er-coap-separate.o \
./apps/er-coap/er-coap-transactions.o \
./apps/er-coap/er-coap.o 

C_DEPS += \
./apps/er-coap/er-coap-block1.d \
./apps/er-coap/er-coap-engine.d \
./apps/er-coap/er-coap-observe-client.d \
./apps/er-coap/er-coap-observe.d \
./apps/er-coap/er-coap-res-well-known-core.d \
./apps/er-coap/er-coap-separate.d \
./apps/er-coap/er-coap-transactions.d \
./apps/er-coap/er-coap.d 


# Each subdirectory must supply rules for building sources it contributes
apps/er-coap/%.o: ../apps/er-coap/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


