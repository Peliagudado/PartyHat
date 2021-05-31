################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

CPP_SRCS += \
../Core/Src/Application.cpp \
../Core/Src/Particles.cpp \
../Core/Src/animations.cpp \
../Core/Src/dsp_window.cpp \
../Core/Src/ws2812b_driver.cpp 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 

OBJS += \
./Core/Src/Application.o \
./Core/Src/Particles.o \
./Core/Src/animations.o \
./Core/Src/dsp_window.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/ws2812b_driver.o 

CPP_DEPS += \
./Core/Src/Application.d \
./Core/Src/Particles.d \
./Core/Src/animations.d \
./Core/Src/dsp_window.d \
./Core/Src/ws2812b_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Application.o: ../Core/Src/Application.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/Application.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/Particles.o: ../Core/Src/Particles.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/Particles.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/animations.o: ../Core/Src/animations.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/animations.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/dsp_window.o: ../Core/Src/dsp_window.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/dsp_window.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l4xx_hal_msp.o: ../Core/Src/stm32l4xx_hal_msp.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l4xx_it.o: ../Core/Src/stm32l4xx_it.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32l4xx.o: ../Core/Src/system_stm32l4xx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32l4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/ws2812b_driver.o: ../Core/Src/ws2812b_driver.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/DSP/Include" -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/EA/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/ws2812b_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

