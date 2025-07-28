################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include platform/subdir.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Src/examples/shared_data/subdir.mk
-include Src/examples/examples_info/subdir.mk
-include Src/examples/example_files/subdir.mk
-include Src/MAC_802_15_8/subdir.mk
-include Src/MAC_802_15_4/subdir.mk
-include Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := UWB_CAN_REVB
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
UWB_CAN_REVB.elf \

MAP_FILES += \
UWB_CAN_REVB.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
UWB_CAN_REVB.list \

OBJCOPY_BIN += \
UWB_CAN_REVB.bin \


# All Target
all: main-build

# Main-build Target
main-build: UWB_CAN_REVB.elf secondary-outputs

# Tool invocations
UWB_CAN_REVB.elf UWB_CAN_REVB.map: $(OBJS) $(USER_OBJS) C:\Users\laure\STM32CubeIDE\workspace_1.11.0\UWB_CAN_REVB\STM32F412RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "UWB_CAN_REVB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\laure\STM32CubeIDE\workspace_1.11.0\UWB_CAN_REVB\STM32F412RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="UWB_CAN_REVB.map" -Wl,--gc-sections -static -L"C:\Users\laure\STM32CubeIDE\workspace_1.11.0\UWB_CAN_REVB\dwt_uwb_driver\lib" -Wl,--whole-archive -ldwt_uwb_driver-m4-hfp-6.0.7 -Wl,--no-whole-archive --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

UWB_CAN_REVB.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "UWB_CAN_REVB.list"
	@echo 'Finished building: $@'
	@echo ' '

UWB_CAN_REVB.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "UWB_CAN_REVB.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) UWB_CAN_REVB.bin UWB_CAN_REVB.elf UWB_CAN_REVB.list UWB_CAN_REVB.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
