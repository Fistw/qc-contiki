################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/collect-view/collect-view-sky.c \
../apps/collect-view/collect-view-template.c \
../apps/collect-view/collect-view-z1.c \
../apps/collect-view/collect-view.c 

OBJS += \
./apps/collect-view/collect-view-sky.o \
./apps/collect-view/collect-view-template.o \
./apps/collect-view/collect-view-z1.o \
./apps/collect-view/collect-view.o 

C_DEPS += \
./apps/collect-view/collect-view-sky.d \
./apps/collect-view/collect-view-template.d \
./apps/collect-view/collect-view-z1.d \
./apps/collect-view/collect-view.d 


# Each subdirectory must supply rules for building sources it contributes
apps/collect-view/%.o: ../apps/collect-view/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


