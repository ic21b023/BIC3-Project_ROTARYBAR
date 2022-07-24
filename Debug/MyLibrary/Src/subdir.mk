################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyLibrary/Src/bargraph.c \
../MyLibrary/Src/rotary.c \
../MyLibrary/Src/serial_protocol.c 

OBJS += \
./MyLibrary/Src/bargraph.o \
./MyLibrary/Src/rotary.o \
./MyLibrary/Src/serial_protocol.o 

C_DEPS += \
./MyLibrary/Src/bargraph.d \
./MyLibrary/Src/rotary.d \
./MyLibrary/Src/serial_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
MyLibrary/Src/%.o: ../MyLibrary/Src/%.c MyLibrary/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

