#!/bin/bash

set +x

AS=./tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc
CC=./tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc

ASFLAGS="-mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant  -x assembler-with-cpp -I./src/main -I./src/main/target -I./src/config/configs/SPEEDYBEEF405V4 -I./src/main/startup/stm32 -I./src/main/drivers/mcu/stm32 -I./lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/inc -I./lib/main/STM32_USB_OTG_Driver/inc -I./lib/main/STM32_USB_Device_Library/Core/inc -I./lib/main/STM32_USB_Device_Library/Class/cdc/inc -I./lib/main/STM32_USB_Device_Library/Class/hid/inc -I./lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper/inc -I./lib/main/STM32_USB_Device_Library/Class/msc/inc -I./lib/main/CMSIS/Core/Include -I./lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx -I./src/main/drivers/mcu/stm32/vcpf4 -I./lib/main/MAVLink -I./src/main/target/STM32F405 -I./lib/main/FatFS -I./lib/main/CMSIS/DSP/Include -I./lib/main/google/olc -MMD -MP"

CFLAGS="-mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant  -I./src/main -I./src/main/target -I./src/config/configs/SPEEDYBEEF405V4 -I./src/main/startup/stm32 -I./src/main/drivers/mcu/stm32 -I./lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/inc -I./lib/main/STM32_USB_OTG_Driver/inc -I./lib/main/STM32_USB_Device_Library/Core/inc -I./lib/main/STM32_USB_Device_Library/Class/cdc/inc -I./lib/main/STM32_USB_Device_Library/Class/hid/inc -I./lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper/inc -I./lib/main/STM32_USB_Device_Library/Class/msc/inc -I./lib/main/CMSIS/Core/Include -I./lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx -I./src/main/drivers/mcu/stm32/vcpf4 -I./lib/main/MAVLink -I./src/main/target/STM32F405 -I./lib/main/FatFS -I./lib/main/CMSIS/DSP/Include -I./lib/main/google/olc  -std=gnu17 -Wall -Wextra -Werror -Wunsafe-loop-optimizations -Wdouble-promotion -Wold-style-definition -ffunction-sections -fdata-sections -fno-common  -DSTM32F40_41xxx -DSTM32F405xx -DHSE_VALUE=8000000 -DSTM32 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4 -DTARGET_FLASH_SIZE=1024 -DHSE_VALUE=8000000 -D_GNU_SOURCE -DUSE_STDPERIPH_DRIVER -DSTM32F405 -DSTM32F405 -DSTM32F4  -DUSE_CONFIG -D __FORKNAME__=\"betaflight\" -D __TARGET__=\"STM32F405\" -D __REVISION__=\"983b51018\" -D __CONFIG_REVISION__=\"9a8694f\" -pipe -MMD -MP  -flto=auto -fuse-linker-plugin -ffast-math -fmerge-all-constants"

ASFILES=( ./src/main/startup/stm32/startup_stm32f40xx.s \
	  lib/main/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.S )

CFILES_OFAST=(src/main/drivers/accgyro/accgyro_mpu.c \
	     src/main/drivers/pwm_output_dshot_shared.c \  
	     src/main/common/encoding.c \  
	     src/main/common/filter.c \  
	     src/main/common/maths.c \  
	     src/main/common/pwl.c \  
	     src/main/common/sdft.c \  
	     src/main/common/stopwatch.c \  
	     src/main/common/typeconversion.c \  
	     src/main/common/vector.c \  
	     src/main/drivers/adc.c \  
	     src/main/drivers/buf_writer.c \  
	     src/main/drivers/bus.c \  
	     src/main/drivers/bus_quadspi.c \  
	     src/main/drivers/bus_spi.c \  
	     src/main/drivers/io.c \  
	     src/main/drivers/serial.c \  
	     src/main/drivers/serial_uart.c \  
	     src/main/drivers/system.c \  
	     src/main/fc/tasks.c \  
	     src/main/fc/runtime_config.c \  
	     src/main/scheduler/scheduler.c \  
	     src/main/fc/core.c \  
	     src/main/fc/rc.c \  
	     src/main/fc/rc_controls.c \  
	     src/main/flight/dyn_notch_filter.c \  
	     src/main/flight/imu.c \  
	     src/main/flight/mixer.c \  
	     src/main/flight/pid.c \  
	     src/main/flight/rpm_filter.c \  
	     src/main/rx/ibus.c \  
	     src/main/rx/frsky_crc.c \  
	     src/main/rx/rc_stats.c \  
	     src/main/rx/rx.c \  
	     src/main/rx/rx_spi.c \  
	     src/main/rx/crsf.c \  
	     src/main/rx/sbus.c \  
	     src/main/rx/sbus_channels.c \  
	     src/main/rx/spektrum.c \  
	     src/main/rx/srxl2.c \  
	     src/main/rx/sumd.c \  
	     src/main/rx/xbus.c \  
	     src/main/rx/fport.c \  
	     src/main/sensors/acceleration.c \  
	     src/main/sensors/boardalignment.c \  
	     src/main/sensors/gyro.c \  
	     src/main/drivers/accgyro/accgyro_mpu3050.c \  
	     src/main/drivers/accgyro/accgyro_spi_bmi160.c \  
	     src/main/drivers/accgyro/accgyro_spi_bmi270.c \  
	     src/main/drivers/accgyro/accgyro_spi_lsm6dso.c \  
	     src/main/drivers/max7456.c )

CFILES_OSIZE=(src/main/drivers/inverter.c \
		src/main/main.c \
		src/main/config/config_eeprom.c \
		src/main/config/config_streamer.c \
		src/main/config/feature.c \
		src/main/config/simplified_tuning.c \
		src/main/cli/cli.c \
		src/main/cli/settings.c \
		src/main/drivers/bus_i2c_config.c \
		src/main/drivers/bus_spi_config.c \
		src/main/drivers/bus_spi_pinconfig.c \
		src/main/drivers/serial_pinconfig.c \
		src/main/drivers/serial_uart_pinconfig.c \
		src/main/fc/board_info.c \
		src/main/io/serial.c \
		src/main/io/transponder_ir.c \
		src/main/io/usb_cdc_hid.c \
		src/main/msp/msp_serial.c \
		src/main/fc/init.c \
		src/main/flight/mixer_init.c \
		src/main/flight/pid_init.c \   
	    src/main/io/serial_4way.c \   
	    src/main/io/serial_4way_avrootloader.c \   
	    src/main/io/serial_4way_stk500v2.c \   
	    src/main/rx/rx_bind.c \   
	    src/main/io/spektrum_vtx_control.c \   
	    src/main/sensors/acceleration_init.c \   
	    src/main/sensors/gyro_init.c \   
	    src/main/cms/cms.c \   
	    src/main/cms/cms_menu_blackbox.c \   
	    src/main/cms/cms_menu_failsafe.c \   
	    src/main/cms/cms_menu_firmware.c \   
	    src/main/cms/cms_menu_gps_rescue.c \   
	    src/main/cms/cms_menu_gps_lap_timer.c \   
	    src/main/cms/cms_menu_imu.c \   
	    src/main/cms/cms_menu_ledstrip.c \   
	    src/main/cms/cms_menu_main.c \   
	    src/main/cms/cms_menu_misc.c \   
	    src/main/cms/cms_menu_osd.c \   
	    src/main/cms/cms_menu_power.c \   
	    src/main/cms/cms_menu_saveexit.c \   
	    src/main/cms/cms_menu_vtx_common.c \   
	    src/main/cms/cms_menu_vtx_rtc6705.c \   
	    src/main/cms/cms_menu_vtx_smartaudio.c \   
	    src/main/cms/cms_menu_vtx_tramp.c \   
	    src/main/cms/cms_menu_persistent_stats.c \   
	    src/main/cms/cms_menu_rpm_limit.c \   
	    src/main/cms/cms_menu_quick.c \   
	    src/main/drivers/display_ug2864hsweg01.c \   
	    src/main/drivers/light_ws2811strip.c \   
	    src/main/drivers/serial_escserial.c \   
	    src/main/drivers/vtx_common.c \   
	    src/main/io/dashboard.c \   
	    src/main/osd/osd.c \   
	    src/main/osd/osd_elements.c \   
	    src/main/osd/osd_warnings.c \   
	    src/main/io/vtx.c \   
	    src/main/io/vtx_rtc6705.c \   
	    src/main/io/vtx_smartaudio.c \   
	    src/main/io/vtx_tramp.c \   
	    src/main/io/vtx_control.c \   
	    src/main/io/vtx_msp.c \   
	    src/main/cms/cms_menu_vtx_msp.c \   
	    src/main/drivers/accgyro/accgyro_mpu6050.c \   
	    src/main/drivers/accgyro/accgyro_mpu6500.c \   
	    src/main/drivers/accgyro/accgyro_spi_icm20689.c \   
	    src/main/drivers/accgyro/accgyro_spi_icm426xx.c \   
	    src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.c \   
	    src/main/drivers/accgyro/accgyro_spi_mpu6000.c \   
	    src/main/drivers/accgyro/accgyro_spi_mpu6500.c \   
	    src/main/drivers/accgyro/accgyro_spi_mpu9250.c \   
	    src/main/drivers/barometer/barometer_2smpb_02b.c \   
	    src/main/drivers/barometer/barometer_bmp085.c \   
	    src/main/drivers/barometer/barometer_bmp280.c \   
	    src/main/drivers/barometer/barometer_lps.c \   
	    src/main/drivers/barometer/barometer_ms5611.c \   
	    src/main/drivers/barometer/barometer_qmp6988.c \   
	    src/main/drivers/compass/compass_ak8963.c \   
	    src/main/drivers/compass/compass_ak8975.c \   
	    src/main/drivers/compass/compass_hmc5883l.c \   
	    src/main/drivers/compass/compass_ist8310.c \   
	    src/main/drivers/compass/compass_lis2mdl.c \   
	    src/main/drivers/compass/compass_lis3mdl.c \   
	    src/main/drivers/compass/compass_qmc5883l.c \   
	    src/main/drivers/vtx_rtc6705.c \   
	    src/main/drivers/vtx_rtc6705_soft_spi.c \   
	    lib/main/google/olc/olc.c)

CFILES_O2=(src/main/drivers/dshot_bitbang_decode.c \
src/main/drivers/mcu/stm32/pwm_output_dshot.c \
src/main/drivers/mcu/stm32/adc_stm32f4xx.c \
src/main/drivers/mcu/stm32/bus_i2c_stm32f4xx.c \
src/main/drivers/mcu/stm32/bus_spi_stdperiph.c \
src/main/drivers/mcu/stm32/debug.c \
src/main/drivers/mcu/stm32/dma_reqmap_mcu.c \
src/main/drivers/mcu/stm32/dma_stm32f4xx.c \
src/main/drivers/mcu/stm32/dshot_bitbang.c \
src/main/drivers/mcu/stm32/dshot_bitbang_stdperiph.c \
src/main/drivers/mcu/stm32/exti.c \
src/main/drivers/mcu/stm32/io_stm32.c \
src/main/drivers/mcu/stm32/light_ws2811strip_stdperiph.c \
src/main/drivers/mcu/stm32/persistent.c \
src/main/drivers/mcu/stm32/pwm_output.c \
src/main/drivers/mcu/stm32/rcc_stm32.c \
src/main/drivers/mcu/stm32/sdio_f4xx.c \
src/main/drivers/mcu/stm32/serial_uart_stdperiph.c \
src/main/drivers/mcu/stm32/serial_uart_stm32f4xx.c \
src/main/drivers/mcu/stm32/system_stm32f4xx.c \
src/main/drivers/mcu/stm32/timer_stdperiph.c \
src/main/drivers/mcu/stm32/timer_stm32f4xx.c \
src/main/drivers/mcu/stm32/transponder_ir_io_stdperiph.c \
src/main/drivers/mcu/stm32/usbd_msc_desc.c \
src/main/drivers/mcu/stm32/camera_control.c \
src/main/startup/stm32/system_stm32f4xx.c \
src/main/drivers/flash/flash.c \
src/main/drivers/flash/flash_m25p16.c \
src/main/drivers/flash/flash_w25m.c \
src/main/drivers/flash/flash_w25n.c \
src/main/drivers/flash/flash_w25q128fv.c \
src/main/io/flashfs.c \
src/main/drivers/usb_msc_common.c \
src/main/drivers/mcu/stm32/usb_msc_f4xx.c \
src/main/msc/usbd_storage.c \
src/main/msc/usbd_storage_emfat.c \
src/main/msc/emfat.c \
src/main/msc/emfat_file.c \
src/main/msc/usbd_storage_sd_spi.c \
src/main/msc/usbd_storage_sdio.c \
src/main/drivers/sdcard.c \
src/main/drivers/sdcard_spi.c \
src/main/drivers/sdcard_sdio_baremetal.c \
src/main/drivers/sdcard_standard.c \
src/main/io/asyncfatfs/asyncfatfs.c \
src/main/io/asyncfatfs/fat_standard.c \
src/main/build/build_config.c \
src/main/build/debug.c \
src/main/build/debug_pin.c \
src/main/build/version.c \
src/main/pg/adc.c \
src/main/pg/alt_hold.c \
src/main/pg/beeper.c \
src/main/pg/beeper_dev.c \
src/main/pg/board.c \
src/main/pg/bus_i2c.c \
src/main/pg/bus_quadspi.c \
src/main/pg/bus_spi.c \
src/main/pg/dashboard.c \
src/main/pg/displayport_profiles.c \
src/main/pg/dyn_notch.c \
src/main/pg/flash.c \
src/main/pg/gps.c \
src/main/pg/gps_lap_timer.c \
src/main/pg/gps_rescue.c \
src/main/pg/gyrodev.c \
src/main/pg/max7456.c \
src/main/pg/mco.c \
src/main/pg/motor.c \
src/main/pg/msp.c \
src/main/pg/pg.c \
src/main/pg/piniobox.c \
src/main/pg/pinio.c \
src/main/pg/pin_pull_up_down.c \
src/main/pg/rcdevice.c \
src/main/pg/rpm_filter.c \
src/main/pg/rx.c \
src/main/pg/rx_pwm.c \
src/main/pg/rx_spi.c \
src/main/pg/rx_spi_cc2500.c \
src/main/pg/rx_spi_expresslrs.c \
src/main/pg/scheduler.c \
src/main/pg/sdcard.c \
src/main/pg/sdio.c \
src/main/pg/serial_uart.c \
src/main/pg/stats.c \
src/main/pg/timerio.c \
src/main/pg/timerup.c \
src/main/pg/usb.c \
src/main/pg/vcd.c \
src/main/pg/vtx_io.c \
src/main/pg/vtx_table.c \
src/main/common/bitarray.c \
src/main/common/colorconversion.c \
src/main/common/crc.c \
src/main/common/explog_approx.c \
src/main/common/gps_conversion.c \
src/main/common/huffman.c \
src/main/common/huffman_table.c \
src/main/common/printf.c \
src/main/common/printf_serial.c \
src/main/common/sensor_alignment.c \
src/main/common/streambuf.c \
src/main/common/string_light.c \
src/main/common/strtol.c \
src/main/common/time.c \
src/main/common/uvarint.c \
src/main/config/config.c \
src/main/drivers/dshot.c \
src/main/drivers/dshot_dpwm.c \
src/main/drivers/dshot_command.c \
src/main/drivers/bus_i2c_busdev.c \
src/main/drivers/bus_i2c_utils.c \
src/main/drivers/bus_i2c_soft.c \
src/main/drivers/bus_octospi.c \
src/main/drivers/buttons.c \
src/main/drivers/display.c \
src/main/drivers/display_canvas.c \
src/main/drivers/dma_common.c \
src/main/drivers/light_led.c \
src/main/drivers/mco.c \
src/main/drivers/motor.c \
src/main/drivers/pinio.c \
src/main/drivers/pin_pull_up_down.c \
src/main/drivers/resource.c \
src/main/drivers/sound_beeper.c \
src/main/drivers/stack_check.c \
src/main/drivers/timer_common.c \
src/main/drivers/transponder_ir_arcitimer.c \
src/main/drivers/transponder_ir_ilap.c \
src/main/drivers/transponder_ir_erlt.c \
src/main/fc/dispatch.c \
src/main/fc/hardfaults.c \
src/main/fc/stats.c \
src/main/io/beeper.c \
src/main/io/piniobox.c \
src/main/io/smartaudio_protocol.c \
src/main/io/statusindicator.c \
src/main/io/tramp_protocol.c \
src/main/io/usb_msc.c \
src/main/msp/msp.c \
src/main/msp/msp_box.c \
src/main/msp/msp_build_info.c \
src/main/sensors/adcinternal.c \
src/main/sensors/battery.c \
src/main/sensors/current.c \
src/main/sensors/voltage.c \
src/main/target/config_helper.c \
src/main/fc/controlrate_profile.c \
src/main/drivers/accgyro/gyro_sync.c \
src/main/drivers/rx/rx_spi.c \
src/main/drivers/rx/rx_xn297.c \
src/main/drivers/rx/rx_pwm.c \
src/main/drivers/serial_softserial.c \
src/main/fc/rc_adjustments.c \
src/main/fc/rc_modes.c \
src/main/flight/position.c \
src/main/flight/failsafe.c \
src/main/flight/gps_rescue.c \
src/main/fc/gps_lap_timer.c \
src/main/flight/alt_hold.c \
src/main/flight/mixer_tricopter.c \
src/main/flight/servos.c \
src/main/flight/servos_tricopter.c \
src/main/rx/jetiexbus.c \
src/main/rx/msp.c \
src/main/rx/pwm.c \
src/main/rx/rx_spi_common.c \
src/main/rx/ghst.c \
src/main/io/spektrum_rssi.c \
src/main/rx/sumh.c \
src/main/rx/msp_override.c \
src/main/sensors/compass.c \
src/main/sensors/initialisation.c \
src/main/blackbox/blackbox.c \
src/main/blackbox/blackbox_encoding.c \
src/main/blackbox/blackbox_io.c \
src/main/drivers/rangefinder/rangefinder_hcsr04.c \
src/main/drivers/rangefinder/rangefinder_lidartf.c \
src/main/drivers/vtx_table.c \
src/main/io/displayport_frsky_osd.c \
src/main/io/displayport_max7456.c \
src/main/io/displayport_msp.c \
src/main/io/displayport_oled.c \
src/main/io/displayport_srxl.c \
src/main/io/displayport_crsf.c \
src/main/io/displayport_hott.c \
src/main/io/frsky_osd.c \
src/main/io/rcdevice_cam.c \
src/main/io/rcdevice.c \
src/main/io/gps.c \
src/main/io/ledstrip.c \
src/main/io/pidaudio.c \
src/main/sensors/barometer.c \
src/main/sensors/rangefinder.c \
src/main/telemetry/telemetry.c \
src/main/telemetry/crsf.c \
src/main/telemetry/ghst.c \
src/main/telemetry/srxl.c \
src/main/telemetry/frsky_hub.c \
src/main/telemetry/hott.c \
src/main/telemetry/jetiexbus.c \
src/main/telemetry/smartport.c \
src/main/telemetry/ltm.c \
src/main/telemetry/mavlink.c \
src/main/telemetry/msp_shared.c \
src/main/telemetry/ibus.c \
src/main/telemetry/ibus_shared.c \
src/main/sensors/esc_sensor.c \
src/main/drivers/accgyro/accgyro_spi_icm20649.c \
src/main/drivers/accgyro/accgyro_spi_l3gd20.c \
src/main/drivers/accgyro/accgyro_spi_lsm6dsv16x.c \
src/main/drivers/accgyro/accgyro_virtual.c \
lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
src/main/drivers/barometer/barometer_bmp388.c \
src/main/drivers/barometer/barometer_dps310.c \
src/main/drivers/barometer/barometer_lps22df.c \
src/main/drivers/barometer/barometer_virtual.c \
src/main/drivers/compass/compass_mpu925x_ak8963.c \
src/main/drivers/compass/compass_virtual.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/misc.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dfsdm.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c \
lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
lib/main/STM32_USB_OTG_Driver/src/usb_core.c \
lib/main/STM32_USB_OTG_Driver/src/usb_dcd.c \
lib/main/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
lib/main/STM32_USB_Device_Library/Core/src/usbd_core.c \
lib/main/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
lib/main/STM32_USB_Device_Library/Core/src/usbd_req.c \
lib/main/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \
lib/main/STM32_USB_Device_Library/Class/hid/src/usbd_hid_core.c \
lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper/src/usbd_hid_cdc_wrapper.c \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_bot.c \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_core.c \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_data.c \
lib/main/STM32_USB_Device_Library/Class/msc/src/usbd_msc_scsi.c \
src/main/drivers/rx/expresslrs_driver.c \
src/main/drivers/rx/rx_a7105.c \
src/main/drivers/rx/rx_cc2500.c \
src/main/drivers/rx/rx_cyrf6936.c \
src/main/drivers/rx/rx_nrf24l01.c \
src/main/drivers/rx/rx_sx127x.c \
src/main/drivers/rx/rx_sx1280.c \
src/main/rx/cc2500_common.c \
src/main/rx/cc2500_frsky_shared.c \
src/main/rx/cc2500_frsky_d.c \
src/main/rx/cc2500_frsky_x.c \
src/main/rx/cc2500_sfhss.c \
src/main/rx/cc2500_redpine.c \
src/main/rx/a7105_flysky.c \
src/main/rx/cyrf6936_spektrum.c \
src/main/rx/expresslrs.c \
src/main/rx/expresslrs_common.c \
src/main/rx/expresslrs_telemetry.c \
src/main/drivers/mcu/stm32/vcpf4/stm32f4xx_it.c \
src/main/drivers/mcu/stm32/vcpf4/usb_bsp.c \
src/main/drivers/mcu/stm32/vcpf4/usbd_desc.c \
src/main/drivers/mcu/stm32/vcpf4/usbd_usr.c \
src/main/drivers/mcu/stm32/vcpf4/usbd_cdc_vcp.c \
src/main/drivers/mcu/stm32/vcpf4/usb_cdc_hid.c \
src/main/drivers/mcu/stm32/serial_usb_vcp.c \
src/main/drivers/usb_io.c)

for fis in ${ASFILES[*]} 
do
    newf=${fis%.[sS]}.o
    set -x
    $AS -c $ASFLAGS $fis -o $newf
    set +x
done

for fis in ${CFILES_OFAST[*]} 
do
    newf=${fis%.c}.o
    set -x
    $CC -c $CFLAGS -Ofast $fis -o $newf
    set +x
done

for fis in ${CFILES_OSIZE[*]} 
do
    newf=${fis%.c}.o
    set -x
    $CC -c $CFLAGS -Os $fis -o $newf
    set +x
done

for fis in ${CFILES_O2[*]} 
do
    newf=${fis%.c}.o
    set -x
    $CC -c $CFLAGS -O2 $fis -o $newf
    set +x
done