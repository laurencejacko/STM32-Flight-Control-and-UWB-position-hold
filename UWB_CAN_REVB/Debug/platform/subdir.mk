################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/deca_mutex.c \
../platform/deca_probe_interface.c \
../platform/deca_sleep.c \
../platform/deca_spi.c \
../platform/port.c 

OBJS += \
./platform/deca_mutex.o \
./platform/deca_probe_interface.o \
./platform/deca_sleep.o \
./platform/deca_spi.o \
./platform/port.o 

C_DEPS += \
./platform/deca_mutex.d \
./platform/deca_probe_interface.d \
./platform/deca_sleep.d \
./platform/deca_spi.d \
./platform/port.d 


# Each subdirectory must supply rules for building sources it contributes
platform/%.o platform/%.su platform/%.cyclo: ../platform/%.c platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/examples_info" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/shared_data" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_4" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_8" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/example_files" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/platform" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/dwt_uwb_driver/Inc" -I"C:/ST/STM32CubeIDE_1.11.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.win32_1.1.100.202309141235/tools/arm-none-eabi/include/sys" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-platform

clean-platform:
	-$(RM) ./platform/deca_mutex.cyclo ./platform/deca_mutex.d ./platform/deca_mutex.o ./platform/deca_mutex.su ./platform/deca_probe_interface.cyclo ./platform/deca_probe_interface.d ./platform/deca_probe_interface.o ./platform/deca_probe_interface.su ./platform/deca_sleep.cyclo ./platform/deca_sleep.d ./platform/deca_sleep.o ./platform/deca_sleep.su ./platform/deca_spi.cyclo ./platform/deca_spi.d ./platform/deca_spi.o ./platform/deca_spi.su ./platform/port.cyclo ./platform/port.d ./platform/port.o ./platform/port.su

.PHONY: clean-platform

