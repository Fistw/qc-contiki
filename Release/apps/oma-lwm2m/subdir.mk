################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/oma-lwm2m/lwm2m-device.c \
../apps/oma-lwm2m/lwm2m-engine.c \
../apps/oma-lwm2m/lwm2m-json.c \
../apps/oma-lwm2m/lwm2m-object.c \
../apps/oma-lwm2m/lwm2m-plain-text.c \
../apps/oma-lwm2m/lwm2m-security.c \
../apps/oma-lwm2m/lwm2m-server.c \
../apps/oma-lwm2m/oma-tlv-reader.c \
../apps/oma-lwm2m/oma-tlv-writer.c \
../apps/oma-lwm2m/oma-tlv.c 

OBJS += \
./apps/oma-lwm2m/lwm2m-device.o \
./apps/oma-lwm2m/lwm2m-engine.o \
./apps/oma-lwm2m/lwm2m-json.o \
./apps/oma-lwm2m/lwm2m-object.o \
./apps/oma-lwm2m/lwm2m-plain-text.o \
./apps/oma-lwm2m/lwm2m-security.o \
./apps/oma-lwm2m/lwm2m-server.o \
./apps/oma-lwm2m/oma-tlv-reader.o \
./apps/oma-lwm2m/oma-tlv-writer.o \
./apps/oma-lwm2m/oma-tlv.o 

C_DEPS += \
./apps/oma-lwm2m/lwm2m-device.d \
./apps/oma-lwm2m/lwm2m-engine.d \
./apps/oma-lwm2m/lwm2m-json.d \
./apps/oma-lwm2m/lwm2m-object.d \
./apps/oma-lwm2m/lwm2m-plain-text.d \
./apps/oma-lwm2m/lwm2m-security.d \
./apps/oma-lwm2m/lwm2m-server.d \
./apps/oma-lwm2m/oma-tlv-reader.d \
./apps/oma-lwm2m/oma-tlv-writer.d \
./apps/oma-lwm2m/oma-tlv.d 


# Each subdirectory must supply rules for building sources it contributes
apps/oma-lwm2m/%.o: ../apps/oma-lwm2m/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


