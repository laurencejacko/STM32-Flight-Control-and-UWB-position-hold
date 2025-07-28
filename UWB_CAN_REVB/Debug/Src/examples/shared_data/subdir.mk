################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/shared_data/shared_functions.c 

OBJS += \
./Src/examples/shared_data/shared_functions.o 

C_DEPS += \
./Src/examples/shared_data/shared_functions.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/shared_data/%.o Src/examples/shared_data/%.su Src/examples/shared_data/%.cyclo: ../Src/examples/shared_data/%.c Src/examples/shared_data/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/examples_info" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/shared_data" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_4" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_8" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/example_files" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/platform" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/dwt_uwb_driver/Inc" -I"C:/ST/STM32CubeIDE_1.11.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.win32_1.1.100.202309141235/tools/arm-none-eabi/include/sys" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-shared_data

clean-Src-2f-examples-2f-shared_data:
	-$(RM) ./Src/examples/shared_data/shared_functions.cyclo ./Src/examples/shared_data/shared_functions.d ./Src/examples/shared_data/shared_functions.o ./Src/examples/shared_data/shared_functions.su

.PHONY: clean-Src-2f-examples-2f-shared_data

