################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/httpd-ws/httpd-ws.c 

OBJS += \
./apps/httpd-ws/httpd-ws.o 

C_DEPS += \
./apps/httpd-ws/httpd-ws.d 


# Each subdirectory must supply rules for building sources it contributes
apps/httpd-ws/%.o: ../apps/httpd-ws/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


