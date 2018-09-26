################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32BG13P/GCC/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/emdrv/gpiointerrupt/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/subdir.mk
-include lcdGraphics/glib/subdir.mk
-include lcdGraphics/drivers/subdir.mk
-include lcdGraphics/dmd/subdir.mk
-include lcdGraphics/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: BLE_Server.axf

# Tool invocations
BLE_Server.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\efr32bg13p632f512gm48.ld" -Xlinker --gc-sections -Xlinker -Map="BLE_Server.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -u _printf_float -o BLE_Server.axf -Wl,--start-group "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o" "./platform/Device/SiliconLabs/EFR32BG13P/Source/system_efr32bg13p.o" "./lcdGraphics/glib/bmp.o" "./lcdGraphics/glib/glib.o" "./lcdGraphics/glib/glib_bitmap.o" "./lcdGraphics/glib/glib_circle.o" "./lcdGraphics/glib/glib_font_narrow_6x8.o" "./lcdGraphics/glib/glib_font_normal_8x8.o" "./lcdGraphics/glib/glib_font_number_16x20.o" "./lcdGraphics/glib/glib_line.o" "./lcdGraphics/glib/glib_polygon.o" "./lcdGraphics/glib/glib_rectangle.o" "./lcdGraphics/glib/glib_string.o" "./lcdGraphics/drivers/display.o" "./lcdGraphics/drivers/displayls013b7dh03.o" "./lcdGraphics/drivers/displaypalemlib.o" "./lcdGraphics/dmd/dmd_display.o" "./lcdGraphics/graphics.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/si7013.o" "./hardware/kit/common/drivers/tempsens.o" "./hardware/kit/common/drivers/udelay.o" "./hardware/kit/common/bsp/bsp_stk.o" "./application_properties.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./main.o" "./pti.o" "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\protocol\bluetooth\lib\EFR32BG13P\GCC\binapploader.o" "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\platform\radio\rail_lib\autogen\librail_release\librail_efr32xg13_gcc_release.a" "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\protocol\bluetooth\lib\EFR32BG13P\GCC\libpsstore.a" "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\protocol\bluetooth\lib\EFR32BG13P\GCC\libbluetooth.a" "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Server\protocol\bluetooth\lib\EFR32BG13P\GCC\libmbedtls.a" -lm "./hardware/kit/common/drivers/retargetio.o" "./hardware/kit/common/drivers/retargetserial.o" "./lcdGraphics/lcd_driver.o" "./platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: BLE_Server.hex'
	arm-none-eabi-objcopy -O ihex "BLE_Server.axf" "BLE_Server.hex"
	@echo ' '

	@echo 'Building bin file: BLE_Server.bin'
	arm-none-eabi-objcopy -O binary "BLE_Server.axf" "BLE_Server.bin"
	@echo ' '

	@echo 'Building s37 file: BLE_Server.s37'
	arm-none-eabi-objcopy -O srec "BLE_Server.axf" "BLE_Server.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "BLE_Server.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) BLE_Server.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
