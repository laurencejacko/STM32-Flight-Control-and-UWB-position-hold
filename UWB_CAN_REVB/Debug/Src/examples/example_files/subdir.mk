################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/example_files/ack_data_rx.c \
../Src/examples/example_files/ack_data_tx.c \
../Src/examples/example_files/bandwidth_calibration.c \
../Src/examples/example_files/continuous_frame.c \
../Src/examples/example_files/continuous_wave.c \
../Src/examples/example_files/double_buffer_rx.c \
../Src/examples/example_files/ds_twr_initiator_sts.c \
../Src/examples/example_files/ds_twr_responder_sts.c \
../Src/examples/example_files/ds_twr_sts_sdc_initiator.c \
../Src/examples/example_files/ds_twr_sts_sdc_responder.c \
../Src/examples/example_files/gpio_example.c \
../Src/examples/example_files/le_pend_rx.c \
../Src/examples/example_files/le_pend_tx.c \
../Src/examples/example_files/otp_write.c \
../Src/examples/example_files/pll_cal.c \
../Src/examples/example_files/read_dev_id.c \
../Src/examples/example_files/rx_diagnostics.c \
../Src/examples/example_files/rx_send_resp.c \
../Src/examples/example_files/rx_sniff.c \
../Src/examples/example_files/rx_with_xtal_trim.c \
../Src/examples/example_files/simple_aes.c \
../Src/examples/example_files/simple_rx_aes.c \
../Src/examples/example_files/simple_rx_nlos.c \
../Src/examples/example_files/simple_rx_pdoa.c \
../Src/examples/example_files/simple_rx_sts_sdc.c \
../Src/examples/example_files/simple_tx.c \
../Src/examples/example_files/simple_tx_aes.c \
../Src/examples/example_files/simple_tx_pdoa.c \
../Src/examples/example_files/simple_tx_sts_sdc.c \
../Src/examples/example_files/spi_crc.c \
../Src/examples/example_files/ss_aes_twr_initiator.c \
../Src/examples/example_files/ss_aes_twr_responder.c \
../Src/examples/example_files/ss_twr_initiator.c \
../Src/examples/example_files/ss_twr_initiator_sts.c \
../Src/examples/example_files/ss_twr_initiator_sts_no_data.c \
../Src/examples/example_files/ss_twr_responder.c \
../Src/examples/example_files/ss_twr_responder_sts.c \
../Src/examples/example_files/ss_twr_responder_sts_no_data.c \
../Src/examples/example_files/timer_example.c \
../Src/examples/example_files/tx_power_adjustment_example.c \
../Src/examples/example_files/tx_sleep.c \
../Src/examples/example_files/tx_sleep_auto.c \
../Src/examples/example_files/tx_sleep_idleRC.c \
../Src/examples/example_files/tx_timed_sleep.c \
../Src/examples/example_files/tx_wait_resp.c \
../Src/examples/example_files/tx_wait_resp_int.c \
../Src/examples/example_files/tx_with_cca.c 

OBJS += \
./Src/examples/example_files/ack_data_rx.o \
./Src/examples/example_files/ack_data_tx.o \
./Src/examples/example_files/bandwidth_calibration.o \
./Src/examples/example_files/continuous_frame.o \
./Src/examples/example_files/continuous_wave.o \
./Src/examples/example_files/double_buffer_rx.o \
./Src/examples/example_files/ds_twr_initiator_sts.o \
./Src/examples/example_files/ds_twr_responder_sts.o \
./Src/examples/example_files/ds_twr_sts_sdc_initiator.o \
./Src/examples/example_files/ds_twr_sts_sdc_responder.o \
./Src/examples/example_files/gpio_example.o \
./Src/examples/example_files/le_pend_rx.o \
./Src/examples/example_files/le_pend_tx.o \
./Src/examples/example_files/otp_write.o \
./Src/examples/example_files/pll_cal.o \
./Src/examples/example_files/read_dev_id.o \
./Src/examples/example_files/rx_diagnostics.o \
./Src/examples/example_files/rx_send_resp.o \
./Src/examples/example_files/rx_sniff.o \
./Src/examples/example_files/rx_with_xtal_trim.o \
./Src/examples/example_files/simple_aes.o \
./Src/examples/example_files/simple_rx_aes.o \
./Src/examples/example_files/simple_rx_nlos.o \
./Src/examples/example_files/simple_rx_pdoa.o \
./Src/examples/example_files/simple_rx_sts_sdc.o \
./Src/examples/example_files/simple_tx.o \
./Src/examples/example_files/simple_tx_aes.o \
./Src/examples/example_files/simple_tx_pdoa.o \
./Src/examples/example_files/simple_tx_sts_sdc.o \
./Src/examples/example_files/spi_crc.o \
./Src/examples/example_files/ss_aes_twr_initiator.o \
./Src/examples/example_files/ss_aes_twr_responder.o \
./Src/examples/example_files/ss_twr_initiator.o \
./Src/examples/example_files/ss_twr_initiator_sts.o \
./Src/examples/example_files/ss_twr_initiator_sts_no_data.o \
./Src/examples/example_files/ss_twr_responder.o \
./Src/examples/example_files/ss_twr_responder_sts.o \
./Src/examples/example_files/ss_twr_responder_sts_no_data.o \
./Src/examples/example_files/timer_example.o \
./Src/examples/example_files/tx_power_adjustment_example.o \
./Src/examples/example_files/tx_sleep.o \
./Src/examples/example_files/tx_sleep_auto.o \
./Src/examples/example_files/tx_sleep_idleRC.o \
./Src/examples/example_files/tx_timed_sleep.o \
./Src/examples/example_files/tx_wait_resp.o \
./Src/examples/example_files/tx_wait_resp_int.o \
./Src/examples/example_files/tx_with_cca.o 

C_DEPS += \
./Src/examples/example_files/ack_data_rx.d \
./Src/examples/example_files/ack_data_tx.d \
./Src/examples/example_files/bandwidth_calibration.d \
./Src/examples/example_files/continuous_frame.d \
./Src/examples/example_files/continuous_wave.d \
./Src/examples/example_files/double_buffer_rx.d \
./Src/examples/example_files/ds_twr_initiator_sts.d \
./Src/examples/example_files/ds_twr_responder_sts.d \
./Src/examples/example_files/ds_twr_sts_sdc_initiator.d \
./Src/examples/example_files/ds_twr_sts_sdc_responder.d \
./Src/examples/example_files/gpio_example.d \
./Src/examples/example_files/le_pend_rx.d \
./Src/examples/example_files/le_pend_tx.d \
./Src/examples/example_files/otp_write.d \
./Src/examples/example_files/pll_cal.d \
./Src/examples/example_files/read_dev_id.d \
./Src/examples/example_files/rx_diagnostics.d \
./Src/examples/example_files/rx_send_resp.d \
./Src/examples/example_files/rx_sniff.d \
./Src/examples/example_files/rx_with_xtal_trim.d \
./Src/examples/example_files/simple_aes.d \
./Src/examples/example_files/simple_rx_aes.d \
./Src/examples/example_files/simple_rx_nlos.d \
./Src/examples/example_files/simple_rx_pdoa.d \
./Src/examples/example_files/simple_rx_sts_sdc.d \
./Src/examples/example_files/simple_tx.d \
./Src/examples/example_files/simple_tx_aes.d \
./Src/examples/example_files/simple_tx_pdoa.d \
./Src/examples/example_files/simple_tx_sts_sdc.d \
./Src/examples/example_files/spi_crc.d \
./Src/examples/example_files/ss_aes_twr_initiator.d \
./Src/examples/example_files/ss_aes_twr_responder.d \
./Src/examples/example_files/ss_twr_initiator.d \
./Src/examples/example_files/ss_twr_initiator_sts.d \
./Src/examples/example_files/ss_twr_initiator_sts_no_data.d \
./Src/examples/example_files/ss_twr_responder.d \
./Src/examples/example_files/ss_twr_responder_sts.d \
./Src/examples/example_files/ss_twr_responder_sts_no_data.d \
./Src/examples/example_files/timer_example.d \
./Src/examples/example_files/tx_power_adjustment_example.d \
./Src/examples/example_files/tx_sleep.d \
./Src/examples/example_files/tx_sleep_auto.d \
./Src/examples/example_files/tx_sleep_idleRC.d \
./Src/examples/example_files/tx_timed_sleep.d \
./Src/examples/example_files/tx_wait_resp.d \
./Src/examples/example_files/tx_wait_resp_int.d \
./Src/examples/example_files/tx_with_cca.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/example_files/%.o Src/examples/example_files/%.su Src/examples/example_files/%.cyclo: ../Src/examples/example_files/%.c Src/examples/example_files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/examples_info" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/shared_data" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_4" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/MAC_802_15_8" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src/examples/example_files" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/Src" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/platform" -I"C:/Users/laure/STM32CubeIDE/workspace_1.11.0/UWB_CAN_REVB/dwt_uwb_driver/Inc" -I"C:/ST/STM32CubeIDE_1.11.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.win32_1.1.100.202309141235/tools/arm-none-eabi/include/sys" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-example_files

clean-Src-2f-examples-2f-example_files:
	-$(RM) ./Src/examples/example_files/ack_data_rx.cyclo ./Src/examples/example_files/ack_data_rx.d ./Src/examples/example_files/ack_data_rx.o ./Src/examples/example_files/ack_data_rx.su ./Src/examples/example_files/ack_data_tx.cyclo ./Src/examples/example_files/ack_data_tx.d ./Src/examples/example_files/ack_data_tx.o ./Src/examples/example_files/ack_data_tx.su ./Src/examples/example_files/bandwidth_calibration.cyclo ./Src/examples/example_files/bandwidth_calibration.d ./Src/examples/example_files/bandwidth_calibration.o ./Src/examples/example_files/bandwidth_calibration.su ./Src/examples/example_files/continuous_frame.cyclo ./Src/examples/example_files/continuous_frame.d ./Src/examples/example_files/continuous_frame.o ./Src/examples/example_files/continuous_frame.su ./Src/examples/example_files/continuous_wave.cyclo ./Src/examples/example_files/continuous_wave.d ./Src/examples/example_files/continuous_wave.o ./Src/examples/example_files/continuous_wave.su ./Src/examples/example_files/double_buffer_rx.cyclo ./Src/examples/example_files/double_buffer_rx.d ./Src/examples/example_files/double_buffer_rx.o ./Src/examples/example_files/double_buffer_rx.su ./Src/examples/example_files/ds_twr_initiator_sts.cyclo ./Src/examples/example_files/ds_twr_initiator_sts.d ./Src/examples/example_files/ds_twr_initiator_sts.o ./Src/examples/example_files/ds_twr_initiator_sts.su ./Src/examples/example_files/ds_twr_responder_sts.cyclo ./Src/examples/example_files/ds_twr_responder_sts.d ./Src/examples/example_files/ds_twr_responder_sts.o ./Src/examples/example_files/ds_twr_responder_sts.su ./Src/examples/example_files/ds_twr_sts_sdc_initiator.cyclo ./Src/examples/example_files/ds_twr_sts_sdc_initiator.d ./Src/examples/example_files/ds_twr_sts_sdc_initiator.o ./Src/examples/example_files/ds_twr_sts_sdc_initiator.su ./Src/examples/example_files/ds_twr_sts_sdc_responder.cyclo ./Src/examples/example_files/ds_twr_sts_sdc_responder.d ./Src/examples/example_files/ds_twr_sts_sdc_responder.o ./Src/examples/example_files/ds_twr_sts_sdc_responder.su ./Src/examples/example_files/gpio_example.cyclo ./Src/examples/example_files/gpio_example.d ./Src/examples/example_files/gpio_example.o ./Src/examples/example_files/gpio_example.su ./Src/examples/example_files/le_pend_rx.cyclo ./Src/examples/example_files/le_pend_rx.d ./Src/examples/example_files/le_pend_rx.o ./Src/examples/example_files/le_pend_rx.su ./Src/examples/example_files/le_pend_tx.cyclo ./Src/examples/example_files/le_pend_tx.d ./Src/examples/example_files/le_pend_tx.o ./Src/examples/example_files/le_pend_tx.su ./Src/examples/example_files/otp_write.cyclo ./Src/examples/example_files/otp_write.d ./Src/examples/example_files/otp_write.o ./Src/examples/example_files/otp_write.su ./Src/examples/example_files/pll_cal.cyclo ./Src/examples/example_files/pll_cal.d ./Src/examples/example_files/pll_cal.o ./Src/examples/example_files/pll_cal.su ./Src/examples/example_files/read_dev_id.cyclo ./Src/examples/example_files/read_dev_id.d ./Src/examples/example_files/read_dev_id.o ./Src/examples/example_files/read_dev_id.su ./Src/examples/example_files/rx_diagnostics.cyclo ./Src/examples/example_files/rx_diagnostics.d ./Src/examples/example_files/rx_diagnostics.o ./Src/examples/example_files/rx_diagnostics.su ./Src/examples/example_files/rx_send_resp.cyclo ./Src/examples/example_files/rx_send_resp.d ./Src/examples/example_files/rx_send_resp.o ./Src/examples/example_files/rx_send_resp.su ./Src/examples/example_files/rx_sniff.cyclo ./Src/examples/example_files/rx_sniff.d ./Src/examples/example_files/rx_sniff.o ./Src/examples/example_files/rx_sniff.su ./Src/examples/example_files/rx_with_xtal_trim.cyclo ./Src/examples/example_files/rx_with_xtal_trim.d ./Src/examples/example_files/rx_with_xtal_trim.o ./Src/examples/example_files/rx_with_xtal_trim.su ./Src/examples/example_files/simple_aes.cyclo ./Src/examples/example_files/simple_aes.d ./Src/examples/example_files/simple_aes.o ./Src/examples/example_files/simple_aes.su ./Src/examples/example_files/simple_rx_aes.cyclo ./Src/examples/example_files/simple_rx_aes.d ./Src/examples/example_files/simple_rx_aes.o ./Src/examples/example_files/simple_rx_aes.su ./Src/examples/example_files/simple_rx_nlos.cyclo ./Src/examples/example_files/simple_rx_nlos.d ./Src/examples/example_files/simple_rx_nlos.o ./Src/examples/example_files/simple_rx_nlos.su ./Src/examples/example_files/simple_rx_pdoa.cyclo ./Src/examples/example_files/simple_rx_pdoa.d ./Src/examples/example_files/simple_rx_pdoa.o ./Src/examples/example_files/simple_rx_pdoa.su ./Src/examples/example_files/simple_rx_sts_sdc.cyclo ./Src/examples/example_files/simple_rx_sts_sdc.d ./Src/examples/example_files/simple_rx_sts_sdc.o ./Src/examples/example_files/simple_rx_sts_sdc.su ./Src/examples/example_files/simple_tx.cyclo ./Src/examples/example_files/simple_tx.d ./Src/examples/example_files/simple_tx.o ./Src/examples/example_files/simple_tx.su ./Src/examples/example_files/simple_tx_aes.cyclo ./Src/examples/example_files/simple_tx_aes.d ./Src/examples/example_files/simple_tx_aes.o ./Src/examples/example_files/simple_tx_aes.su ./Src/examples/example_files/simple_tx_pdoa.cyclo ./Src/examples/example_files/simple_tx_pdoa.d ./Src/examples/example_files/simple_tx_pdoa.o ./Src/examples/example_files/simple_tx_pdoa.su ./Src/examples/example_files/simple_tx_sts_sdc.cyclo ./Src/examples/example_files/simple_tx_sts_sdc.d ./Src/examples/example_files/simple_tx_sts_sdc.o ./Src/examples/example_files/simple_tx_sts_sdc.su ./Src/examples/example_files/spi_crc.cyclo ./Src/examples/example_files/spi_crc.d ./Src/examples/example_files/spi_crc.o ./Src/examples/example_files/spi_crc.su ./Src/examples/example_files/ss_aes_twr_initiator.cyclo ./Src/examples/example_files/ss_aes_twr_initiator.d ./Src/examples/example_files/ss_aes_twr_initiator.o ./Src/examples/example_files/ss_aes_twr_initiator.su ./Src/examples/example_files/ss_aes_twr_responder.cyclo ./Src/examples/example_files/ss_aes_twr_responder.d
	-$(RM) ./Src/examples/example_files/ss_aes_twr_responder.o ./Src/examples/example_files/ss_aes_twr_responder.su ./Src/examples/example_files/ss_twr_initiator.cyclo ./Src/examples/example_files/ss_twr_initiator.d ./Src/examples/example_files/ss_twr_initiator.o ./Src/examples/example_files/ss_twr_initiator.su ./Src/examples/example_files/ss_twr_initiator_sts.cyclo ./Src/examples/example_files/ss_twr_initiator_sts.d ./Src/examples/example_files/ss_twr_initiator_sts.o ./Src/examples/example_files/ss_twr_initiator_sts.su ./Src/examples/example_files/ss_twr_initiator_sts_no_data.cyclo ./Src/examples/example_files/ss_twr_initiator_sts_no_data.d ./Src/examples/example_files/ss_twr_initiator_sts_no_data.o ./Src/examples/example_files/ss_twr_initiator_sts_no_data.su ./Src/examples/example_files/ss_twr_responder.cyclo ./Src/examples/example_files/ss_twr_responder.d ./Src/examples/example_files/ss_twr_responder.o ./Src/examples/example_files/ss_twr_responder.su ./Src/examples/example_files/ss_twr_responder_sts.cyclo ./Src/examples/example_files/ss_twr_responder_sts.d ./Src/examples/example_files/ss_twr_responder_sts.o ./Src/examples/example_files/ss_twr_responder_sts.su ./Src/examples/example_files/ss_twr_responder_sts_no_data.cyclo ./Src/examples/example_files/ss_twr_responder_sts_no_data.d ./Src/examples/example_files/ss_twr_responder_sts_no_data.o ./Src/examples/example_files/ss_twr_responder_sts_no_data.su ./Src/examples/example_files/timer_example.cyclo ./Src/examples/example_files/timer_example.d ./Src/examples/example_files/timer_example.o ./Src/examples/example_files/timer_example.su ./Src/examples/example_files/tx_power_adjustment_example.cyclo ./Src/examples/example_files/tx_power_adjustment_example.d ./Src/examples/example_files/tx_power_adjustment_example.o ./Src/examples/example_files/tx_power_adjustment_example.su ./Src/examples/example_files/tx_sleep.cyclo ./Src/examples/example_files/tx_sleep.d ./Src/examples/example_files/tx_sleep.o ./Src/examples/example_files/tx_sleep.su ./Src/examples/example_files/tx_sleep_auto.cyclo ./Src/examples/example_files/tx_sleep_auto.d ./Src/examples/example_files/tx_sleep_auto.o ./Src/examples/example_files/tx_sleep_auto.su ./Src/examples/example_files/tx_sleep_idleRC.cyclo ./Src/examples/example_files/tx_sleep_idleRC.d ./Src/examples/example_files/tx_sleep_idleRC.o ./Src/examples/example_files/tx_sleep_idleRC.su ./Src/examples/example_files/tx_timed_sleep.cyclo ./Src/examples/example_files/tx_timed_sleep.d ./Src/examples/example_files/tx_timed_sleep.o ./Src/examples/example_files/tx_timed_sleep.su ./Src/examples/example_files/tx_wait_resp.cyclo ./Src/examples/example_files/tx_wait_resp.d ./Src/examples/example_files/tx_wait_resp.o ./Src/examples/example_files/tx_wait_resp.su ./Src/examples/example_files/tx_wait_resp_int.cyclo ./Src/examples/example_files/tx_wait_resp_int.d ./Src/examples/example_files/tx_wait_resp_int.o ./Src/examples/example_files/tx_wait_resp_int.su ./Src/examples/example_files/tx_with_cca.cyclo ./Src/examples/example_files/tx_with_cca.d ./Src/examples/example_files/tx_with_cca.o ./Src/examples/example_files/tx_with_cca.su

.PHONY: clean-Src-2f-examples-2f-example_files

