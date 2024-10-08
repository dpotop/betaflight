#!/bin/bash

set -x

LINKFILES=(src/main/startup/stm32/startup_stm32f40xx.o \
src/main/drivers/accgyro/accgyro_mpu.o \
src/main/drivers/dshot_bitbang_decode.o \
src/main/drivers/inverter.o \
src/main/drivers/pwm_output_dshot_shared.o \
src/main/drivers/mcu/stm32/pwm_output_dshot.o \
src/main/drivers/mcu/stm32/adc_stm32f4xx.o \
src/main/drivers/mcu/stm32/bus_i2c_stm32f4xx.o \
src/main/drivers/mcu/stm32/bus_spi_stdperiph.o \
src/main/drivers/mcu/stm32/debug.o \
src/main/drivers/mcu/stm32/dma_reqmap_mcu.o \
src/main/drivers/mcu/stm32/dma_stm32f4xx.o \
src/main/drivers/mcu/stm32/dshot_bitbang.o \
src/main/drivers/mcu/stm32/dshot_bitbang_stdperiph.o \
src/main/drivers/mcu/stm32/exti.o \
src/main/drivers/mcu/stm32/io_stm32.o \
src/main/drivers/mcu/stm32/light_ws2811strip_stdperiph.o \
src/main/drivers/mcu/stm32/persistent.o \
src/main/drivers/mcu/stm32/pwm_output.o \
src/main/drivers/mcu/stm32/rcc_stm32.o \
src/main/drivers/mcu/stm32/sdio_f4xx.o \
src/main/drivers/mcu/stm32/serial_uart_stdperiph.o \
src/main/drivers/mcu/stm32/serial_uart_stm32f4xx.o \
src/main/drivers/mcu/stm32/system_stm32f4xx.o \
src/main/drivers/mcu/stm32/timer_stdperiph.o \
src/main/drivers/mcu/stm32/timer_stm32f4xx.o \
src/main/drivers/mcu/stm32/transponder_ir_io_stdperiph.o \
src/main/drivers/mcu/stm32/usbd_msc_desc.o \
src/main/drivers/mcu/stm32/camera_control.o \
src/main/startup/stm32/system_stm32f4xx.o \
lib/main/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.o \
src/main/drivers/flash/flash.o \
src/main/drivers/flash/flash_m25p16.o \
src/main/drivers/flash/flash_w25m.o \
src/main/drivers/flash/flash_w25n.o \
src/main/drivers/flash/flash_w25q128fv.o \
src/main/io/flashfs.o \
src/main/drivers/usb_msc_common.o \
src/main/drivers/mcu/stm32/usb_msc_f4xx.o \
src/main/msc/usbd_storage.o \
src/main/msc/usbd_storage_emfat.o \
src/main/msc/emfat.o \
src/main/msc/emfat_file.o \
src/main/msc/usbd_storage_sd_spi.o \
src/main/msc/usbd_storage_sdio.o \
src/main/drivers/sdcard.o \
src/main/drivers/sdcard_spi.o \
src/main/drivers/sdcard_sdio_baremetal.o \
src/main/drivers/sdcard_standard.o \
src/main/io/asyncfatfs/asyncfatfs.o \
src/main/io/asyncfatfs/fat_standard.o \
src/main/build/build_config.o \
src/main/build/debug.o \
src/main/build/debug_pin.o \
src/main/build/version.o \
src/main/main.o \
src/main/pg/adc.o \
src/main/pg/alt_hold.o \
src/main/pg/beeper.o \
src/main/pg/beeper_dev.o \
src/main/pg/board.o \
src/main/pg/bus_i2c.o \
src/main/pg/bus_quadspi.o \
src/main/pg/bus_spi.o \
src/main/pg/dashboard.o \
src/main/pg/displayport_profiles.o \
src/main/pg/dyn_notch.o \
src/main/pg/flash.o \
src/main/pg/gps.o \
src/main/pg/gps_lap_timer.o \
src/main/pg/gps_rescue.o \
src/main/pg/gyrodev.o \
src/main/pg/max7456.o \
src/main/pg/mco.o \
src/main/pg/motor.o \
src/main/pg/msp.o \
src/main/pg/pg.o \
src/main/pg/piniobox.o \
src/main/pg/pinio.o \
src/main/pg/pin_pull_up_down.o \
src/main/pg/rcdevice.o \
src/main/pg/rpm_filter.o \
src/main/pg/rx.o \
src/main/pg/rx_pwm.o \
src/main/pg/rx_spi.o \
src/main/pg/rx_spi_cc2500.o \
src/main/pg/rx_spi_expresslrs.o \
src/main/pg/scheduler.o \
src/main/pg/sdcard.o \
src/main/pg/sdio.o \
src/main/pg/serial_uart.o \
src/main/pg/stats.o \
src/main/pg/timerio.o \
src/main/pg/timerup.o \
src/main/pg/usb.o \
src/main/pg/vcd.o \
src/main/pg/vtx_io.o \
src/main/pg/vtx_table.o \
src/main/common/bitarray.o \
src/main/common/colorconversion.o \
src/main/common/crc.o \
src/main/common/encoding.o \
src/main/common/explog_approx.o \
src/main/common/filter.o \
src/main/common/gps_conversion.o \
src/main/common/huffman.o \
src/main/common/huffman_table.o \
src/main/common/maths.o \
src/main/common/printf.o \
src/main/common/printf_serial.o \
src/main/common/pwl.o \
src/main/common/sdft.o \
src/main/common/sensor_alignment.o \
src/main/common/stopwatch.o \
src/main/common/streambuf.o \
src/main/common/string_light.o \
src/main/common/strtol.o \
src/main/common/time.o \
src/main/common/typeconversion.o \
src/main/common/uvarint.o \
src/main/common/vector.o \
src/main/config/config.o \
src/main/config/config_eeprom.o \
src/main/config/config_streamer.o \
src/main/config/feature.o \
src/main/config/simplified_tuning.o \
src/main/cli/cli.o \
src/main/cli/settings.o \
src/main/drivers/adc.o \
src/main/drivers/dshot.o \
src/main/drivers/dshot_dpwm.o \
src/main/drivers/dshot_command.o \
src/main/drivers/buf_writer.o \
src/main/drivers/bus.o \
src/main/drivers/bus_i2c_config.o \
src/main/drivers/bus_i2c_busdev.o \
src/main/drivers/bus_i2c_utils.o \
src/main/drivers/bus_i2c_soft.o \
src/main/drivers/bus_octospi.o \
src/main/drivers/bus_quadspi.o \
src/main/drivers/bus_spi.o \
src/main/drivers/bus_spi_config.o \
src/main/drivers/bus_spi_pinconfig.o \
src/main/drivers/buttons.o \
src/main/drivers/display.o \
src/main/drivers/display_canvas.o \
src/main/drivers/dma_common.o \
src/main/drivers/io.o \
src/main/drivers/light_led.o \
src/main/drivers/mco.o \
src/main/drivers/motor.o \
src/main/drivers/pinio.o \
src/main/drivers/pin_pull_up_down.o \
src/main/drivers/resource.o \
src/main/drivers/serial.o \
src/main/drivers/serial_pinconfig.o \
src/main/drivers/serial_uart.o \
src/main/drivers/serial_uart_pinconfig.o \
src/main/drivers/sound_beeper.o \
src/main/drivers/stack_check.o \
src/main/drivers/system.o \
src/main/drivers/timer_common.o \
src/main/drivers/transponder_ir_arcitimer.o \
src/main/drivers/transponder_ir_ilap.o \
src/main/drivers/transponder_ir_erlt.o \
src/main/fc/board_info.o \
src/main/fc/dispatch.o \
src/main/fc/hardfaults.o \
src/main/fc/tasks.o \
src/main/fc/runtime_config.o \
src/main/fc/stats.o \
src/main/io/beeper.o \
src/main/io/piniobox.o \
src/main/io/serial.o \
src/main/io/smartaudio_protocol.o \
src/main/io/statusindicator.o \
src/main/io/tramp_protocol.o \
src/main/io/transponder_ir.o \
src/main/io/usb_cdc_hid.o \
src/main/io/usb_msc.o \
src/main/msp/msp.o \
src/main/msp/msp_box.o \
src/main/msp/msp_build_info.o \
src/main/msp/msp_serial.o \
src/main/scheduler/scheduler.o \
src/main/sensors/adcinternal.o \
src/main/sensors/battery.o \
src/main/sensors/current.o \
src/main/sensors/voltage.o \
src/main/target/config_helper.o \
src/main/fc/init.o \
src/main/fc/controlrate_profile.o \
src/main/drivers/accgyro/gyro_sync.o \
src/main/drivers/rx/rx_spi.o \
src/main/drivers/rx/rx_xn297.o \
src/main/drivers/rx/rx_pwm.o \
src/main/drivers/serial_softserial.o \
src/main/fc/core.o \
src/main/fc/rc.o \
src/main/fc/rc_adjustments.o \
src/main/fc/rc_controls.o \
src/main/fc/rc_modes.o \
src/main/flight/position.o \
src/main/flight/failsafe.o \
src/main/flight/gps_rescue.o \
src/main/fc/gps_lap_timer.o \
src/main/flight/dyn_notch_filter.o \
src/main/flight/alt_hold.o \
src/main/flight/imu.o \
src/main/flight/mixer.o \
src/main/flight/mixer_init.o \
src/main/flight/mixer_tricopter.o \
src/main/flight/pid.o \
src/main/flight/pid_init.o \
src/main/flight/rpm_filter.o \
src/main/flight/servos.o \
src/main/flight/servos_tricopter.o \
src/main/io/serial_4way.o \
src/main/io/serial_4way_avrootloader.o \
src/main/io/serial_4way_stk500v2.o \
src/main/rx/ibus.o \
src/main/rx/jetiexbus.o \
src/main/rx/msp.o \
src/main/rx/pwm.o \
src/main/rx/frsky_crc.o \
src/main/rx/rc_stats.o \
src/main/rx/rx.o \
src/main/rx/rx_bind.o \
src/main/rx/rx_spi.o \
src/main/rx/rx_spi_common.o \
src/main/rx/crsf.o \
src/main/rx/ghst.o \
src/main/rx/sbus.o \
src/main/rx/sbus_channels.o \
src/main/rx/spektrum.o \
src/main/rx/srxl2.o \
src/main/io/spektrum_vtx_control.o \
src/main/io/spektrum_rssi.o \
src/main/rx/sumd.o \
src/main/rx/sumh.o \
src/main/rx/xbus.o \
src/main/rx/fport.o \
src/main/rx/msp_override.o \
src/main/sensors/acceleration.o \
src/main/sensors/acceleration_init.o \
src/main/sensors/boardalignment.o \
src/main/sensors/compass.o \
src/main/sensors/gyro.o \
src/main/sensors/gyro_init.o \
src/main/sensors/initialisation.o \
src/main/blackbox/blackbox.o \
src/main/blackbox/blackbox_encoding.o \
src/main/blackbox/blackbox_io.o \
src/main/cms/cms.o \
src/main/cms/cms_menu_blackbox.o \
src/main/cms/cms_menu_failsafe.o \
src/main/cms/cms_menu_firmware.o \
src/main/cms/cms_menu_gps_rescue.o \
src/main/cms/cms_menu_gps_lap_timer.o \
src/main/cms/cms_menu_imu.o \
src/main/cms/cms_menu_ledstrip.o \
src/main/cms/cms_menu_main.o \
src/main/cms/cms_menu_misc.o \
src/main/cms/cms_menu_osd.o \
src/main/cms/cms_menu_power.o \
src/main/cms/cms_menu_saveexit.o \
src/main/cms/cms_menu_vtx_common.o \
src/main/cms/cms_menu_vtx_rtc6705.o \
src/main/cms/cms_menu_vtx_smartaudio.o \
src/main/cms/cms_menu_vtx_tramp.o \
src/main/cms/cms_menu_persistent_stats.o \
src/main/cms/cms_menu_rpm_limit.o \
src/main/cms/cms_menu_quick.o \
src/main/drivers/display_ug2864hsweg01.o \
src/main/drivers/light_ws2811strip.o \
src/main/drivers/rangefinder/rangefinder_hcsr04.o \
src/main/drivers/rangefinder/rangefinder_lidartf.o \
src/main/drivers/serial_escserial.o \
src/main/drivers/vtx_common.o \
src/main/drivers/vtx_table.o \
src/main/io/dashboard.o \
src/main/io/displayport_frsky_osd.o \
src/main/io/displayport_max7456.o \
src/main/io/displayport_msp.o \
src/main/io/displayport_oled.o \
src/main/io/displayport_srxl.o \
src/main/io/displayport_crsf.o \
src/main/io/displayport_hott.o \
src/main/io/frsky_osd.o \
src/main/io/rcdevice_cam.o \
src/main/io/rcdevice.o \
src/main/io/gps.o \
src/main/io/ledstrip.o \
src/main/io/pidaudio.o \
src/main/osd/osd.o \
src/main/osd/osd_elements.o \
src/main/osd/osd_warnings.o \
src/main/sensors/barometer.o \
src/main/sensors/rangefinder.o \
src/main/telemetry/telemetry.o \
src/main/telemetry/crsf.o \
src/main/telemetry/ghst.o \
src/main/telemetry/srxl.o \
src/main/telemetry/frsky_hub.o \
src/main/telemetry/hott.o \
src/main/telemetry/jetiexbus.o \
src/main/telemetry/smartport.o \
src/main/telemetry/ltm.o \
src/main/telemetry/mavlink.o \
src/main/telemetry/msp_shared.o \
src/main/telemetry/ibus.o \
src/main/telemetry/ibus_shared.o \
src/main/sensors/esc_sensor.o \
src/main/io/vtx.o \
src/main/io/vtx_rtc6705.o \
src/main/io/vtx_smartaudio.o \
src/main/io/vtx_tramp.o \
src/main/io/vtx_control.o \
src/main/io/vtx_msp.o \
src/main/cms/cms_menu_vtx_msp.o \
src/main/drivers/accgyro/accgyro_mpu3050.o \
src/main/drivers/accgyro/accgyro_mpu6050.o \
src/main/drivers/accgyro/accgyro_mpu6500.o \
src/main/drivers/accgyro/accgyro_spi_bmi160.o \
src/main/drivers/accgyro/accgyro_spi_bmi270.o \
src/main/drivers/accgyro/accgyro_spi_icm20649.o \
src/main/drivers/accgyro/accgyro_spi_icm20689.o \
src/main/drivers/accgyro/accgyro_spi_icm426xx.o \
src/main/drivers/accgyro/accgyro_spi_l3gd20.o \
src/main/drivers/accgyro/accgyro_spi_lsm6dso.o \
src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.o \
src/main/drivers/accgyro/accgyro_spi_lsm6dsv16x.o \
src/main/drivers/accgyro/accgyro_spi_mpu6000.o \
src/main/drivers/accgyro/accgyro_spi_mpu6500.o \
src/main/drivers/accgyro/accgyro_spi_mpu9250.o \
src/main/drivers/accgyro/accgyro_virtual.o \
lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.o \
src/main/drivers/barometer/barometer_2smpb_02b.o \
src/main/drivers/barometer/barometer_bmp085.o \
src/main/drivers/barometer/barometer_bmp280.o \
src/main/drivers/barometer/barometer_bmp388.o \
src/main/drivers/barometer/barometer_dps310.o \
src/main/drivers/barometer/barometer_lps22df.o \
src/main/drivers/barometer/barometer_lps.o \
src/main/drivers/barometer/barometer_ms5611.o \
src/main/drivers/barometer/barometer_qmp6988.o \
src/main/drivers/barometer/barometer_virtual.o \
src/main/drivers/compass/compass_ak8963.o \
src/main/drivers/compass/compass_ak8975.o \
src/main/drivers/compass/compass_hmc5883l.o \
src/main/drivers/compass/compass_ist8310.o \
src/main/drivers/compass/compass_lis2mdl.o \
src/main/drivers/compass/compass_lis3mdl.o \
src/main/drivers/compass/compass_mpu925x_ak8963.o \
src/main/drivers/compass/compass_qmc5883l.o \
src/main/drivers/compass/compass_virtual.o \
src/main/drivers/max7456.o \
src/main/drivers/vtx_rtc6705.o \
src/main/drivers/vtx_rtc6705_soft_spi.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/misc.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dfsdm.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.o \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.o \
lib/main/STM32_USB_OTG_Driver/src/usb_core.o \
lib/main/STM32_USB_OTG_Driver/src/usb_dcd.o \
lib/main/STM32_USB_OTG_Driver/src/usb_dcd_int.o \
lib/main/STM32_USB_Device_Library/Core/src/usbd_core.o \
lib/main/STM32_USB_Device_Library/Core/src/usbd_ioreq.o \
lib/main/STM32_USB_Device_Library/Core/src/usbd_req.o \
lib/main/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.o \
lib/main/STM32_USB_Device_Library/Class/hid/src/usbd_hid_core.o \
lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper/src/usbd_hid_cdc_wrapper.o \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_bot.o \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_core.o \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_data.o \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_scsi.o \
src/main/drivers/rx/expresslrs_driver.o \
src/main/drivers/rx/rx_a7105.o \
src/main/drivers/rx/rx_cc2500.o \
src/main/drivers/rx/rx_cyrf6936.o \
src/main/drivers/rx/rx_nrf24l01.o \
src/main/drivers/rx/rx_sx127x.o \
src/main/drivers/rx/rx_sx1280.o \
src/main/rx/cc2500_common.o \
src/main/rx/cc2500_frsky_shared.o \
src/main/rx/cc2500_frsky_d.o \
src/main/rx/cc2500_frsky_x.o \
src/main/rx/cc2500_sfhss.o \
src/main/rx/cc2500_redpine.o \
src/main/rx/a7105_flysky.o \
src/main/rx/cyrf6936_spektrum.o \
src/main/rx/expresslrs.o \
src/main/rx/expresslrs_common.o \
src/main/rx/expresslrs_telemetry.o \
src/main/drivers/mcu/stm32/vcpf4/stm32f4xx_it.o \
src/main/drivers/mcu/stm32/vcpf4/usb_bsp.o \
src/main/drivers/mcu/stm32/vcpf4/usbd_desc.o \
src/main/drivers/mcu/stm32/vcpf4/usbd_usr.o \
src/main/drivers/mcu/stm32/vcpf4/usbd_cdc_vcp.o \
src/main/drivers/mcu/stm32/vcpf4/usb_cdc_hid.o \
src/main/drivers/mcu/stm32/serial_usb_vcp.o \
src/main/drivers/usb_io.o \
lib/main/google/olc/olc.o)

# GCC seems needed for link because the application makes use of
# link-time optimizations (which cannot be performed by LD?).
# The compiler flags are here "-flto=auto -fuse-linker-plugin"
# and some other optimization flags make more sense from this
# perspective, such as -fmerge-all-constants -Ofast. But clearly
# I won't be able to have a transparent linking phase.
./tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc \
-o ./betaflight_STM32F405_SPEEDYBEEF405V4.elf \
${LINKFILES[@]} \
-lm -nostartfiles \
--specs=nano.specs -lc \
-lnosys -mthumb -mcpu=cortex-m4 -march=armv7e-m \
-mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -flto=auto -fuse-linker-plugin \
-ffast-math -fmerge-all-constants -Ofast  -static \
-Wl,-gc-sections,-Map,./betaflight_STM32F405_SPEEDYBEEF405V4.map \
-Wl,-L./src/link -Wl,--cref -Wl,--no-wchar-size-warning \
-Wl,--print-memory-usage -T./src/link/stm32_flash_f405.ld


./tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-size ./betaflight_STM32F405_SPEEDYBEEF405V4.elf
./tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-objcopy -O ihex --set-start 0x8000000 ./betaflight_STM32F405_SPEEDYBEEF405V4.elf ./betaflight_4.6.0_STM32F405_SPEEDYBEEF405V4.hex

# Working without the standard libs requires replacing:
#   --specs=nano.specs -lc
# with
#   -nostdlib
# (and don't forget the line continuations).
# To get details on what is done, instead of gcc, use gcc -v
# When first trying to do this, I get no warnings due to the absence
# of, say, malloc and free, only 4 functions are missing:
#    in function `_close_r': _close is not implemented and will always fail
#    in function `_lseek_r': _lseek is not implemented and will always fail
#    in function `_read_r': _read is not implemented and will always fail
#    in function `_write_r': _write is not implemented and will always fail
