################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../apps/antelope/antelope.c \
../apps/antelope/aql-adt.c \
../apps/antelope/aql-exec.c \
../apps/antelope/aql-lexer.c \
../apps/antelope/aql-parser.c \
../apps/antelope/index-inline.c \
../apps/antelope/index-maxheap.c \
../apps/antelope/index-memhash.c \
../apps/antelope/index.c \
../apps/antelope/lvm.c \
../apps/antelope/relation.c \
../apps/antelope/result.c \
../apps/antelope/storage-cfs.c 

OBJS += \
./apps/antelope/antelope.o \
./apps/antelope/aql-adt.o \
./apps/antelope/aql-exec.o \
./apps/antelope/aql-lexer.o \
./apps/antelope/aql-parser.o \
./apps/antelope/index-inline.o \
./apps/antelope/index-maxheap.o \
./apps/antelope/index-memhash.o \
./apps/antelope/index.o \
./apps/antelope/lvm.o \
./apps/antelope/relation.o \
./apps/antelope/result.o \
./apps/antelope/storage-cfs.o 

C_DEPS += \
./apps/antelope/antelope.d \
./apps/antelope/aql-adt.d \
./apps/antelope/aql-exec.d \
./apps/antelope/aql-lexer.d \
./apps/antelope/aql-parser.d \
./apps/antelope/index-inline.d \
./apps/antelope/index-maxheap.d \
./apps/antelope/index-memhash.d \
./apps/antelope/index.d \
./apps/antelope/lvm.d \
./apps/antelope/relation.d \
./apps/antelope/result.d \
./apps/antelope/storage-cfs.d 


# Each subdirectory must supply rules for building sources it contributes
apps/antelope/%.o: ../apps/antelope/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F105RCTx -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


