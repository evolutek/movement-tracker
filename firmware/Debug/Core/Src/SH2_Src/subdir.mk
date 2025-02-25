################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SH2_Src/euler.c \
../Core/Src/SH2_Src/sh2.c \
../Core/Src/SH2_Src/sh2_SensorValue.c \
../Core/Src/SH2_Src/sh2_util.c \
../Core/Src/SH2_Src/shtp.c 

OBJS += \
./Core/Src/SH2_Src/euler.o \
./Core/Src/SH2_Src/sh2.o \
./Core/Src/SH2_Src/sh2_SensorValue.o \
./Core/Src/SH2_Src/sh2_util.o \
./Core/Src/SH2_Src/shtp.o 

C_DEPS += \
./Core/Src/SH2_Src/euler.d \
./Core/Src/SH2_Src/sh2.d \
./Core/Src/SH2_Src/sh2_SensorValue.d \
./Core/Src/SH2_Src/sh2_util.d \
./Core/Src/SH2_Src/shtp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SH2_Src/%.o Core/Src/SH2_Src/%.su Core/Src/SH2_Src/%.cyclo: ../Core/Src/SH2_Src/%.c Core/Src/SH2_Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SH2_Src

clean-Core-2f-Src-2f-SH2_Src:
	-$(RM) ./Core/Src/SH2_Src/euler.cyclo ./Core/Src/SH2_Src/euler.d ./Core/Src/SH2_Src/euler.o ./Core/Src/SH2_Src/euler.su ./Core/Src/SH2_Src/sh2.cyclo ./Core/Src/SH2_Src/sh2.d ./Core/Src/SH2_Src/sh2.o ./Core/Src/SH2_Src/sh2.su ./Core/Src/SH2_Src/sh2_SensorValue.cyclo ./Core/Src/SH2_Src/sh2_SensorValue.d ./Core/Src/SH2_Src/sh2_SensorValue.o ./Core/Src/SH2_Src/sh2_SensorValue.su ./Core/Src/SH2_Src/sh2_util.cyclo ./Core/Src/SH2_Src/sh2_util.d ./Core/Src/SH2_Src/sh2_util.o ./Core/Src/SH2_Src/sh2_util.su ./Core/Src/SH2_Src/shtp.cyclo ./Core/Src/SH2_Src/shtp.d ./Core/Src/SH2_Src/shtp.o ./Core/Src/SH2_Src/shtp.su

.PHONY: clean-Core-2f-Src-2f-SH2_Src

