################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include stack/bt_mesh_stack/subdir.mk
-include src/subdir.mk
-include serial_vcom/subdir.mk
-include kit_lcd/BRD4104A/subdir.mk
-include kit_flashpwr/BRD4104A/subdir.mk
-include kit/BRD4104A/subdir.mk
-include glib/BRD4104A/subdir.mk
-include emlib/subdir.mk
-include emdrv/subdir.mk
-include device/EFR32_B_1_3_P/gcc/subdir.mk
-include device/EFR32_B_1_3_P/subdir.mk
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
all: btmesh-LightsModule.axf

# Tool invocations
btmesh-LightsModule.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\btmesh-LightsModule\linker\GCC\efr32bg13p.ld" -L"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/lib/EFR32XG13X/GCC/" -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="btmesh-LightsModule.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -o btmesh-LightsModule.axf -Wl,--start-group "./dcd_light.o" "./gatt_db.o" "./graphics.o" "./main.o" "./pti.o" "./device/EFR32_B_1_3_P/gcc/startup_efr32bg13p.o" "./device/EFR32_B_1_3_P/system_efr32bg13p.o" "./emdrv/dmadrv.o" "./emdrv/gpiointerrupt.o" "./emdrv/sleep.o" "./emdrv/tempdrv.o" "./emdrv/uartdrv.o" "./emlib/em_adc.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_crypto.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_ldma.o" "./emlib/em_leuart.o" "./emlib/em_msc.o" "./emlib/em_rmu.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_usart.o" "./glib/BRD4104A/bmp.o" "./glib/BRD4104A/dmd_display.o" "./glib/BRD4104A/glib.o" "./glib/BRD4104A/glib_bitmap.o" "./glib/BRD4104A/glib_circle.o" "./glib/BRD4104A/glib_font_narrow_6x8.o" "./glib/BRD4104A/glib_font_normal_8x8.o" "./glib/BRD4104A/glib_font_number_16x20.o" "./glib/BRD4104A/glib_line.o" "./glib/BRD4104A/glib_polygon.o" "./glib/BRD4104A/glib_rectangle.o" "./glib/BRD4104A/glib_string.o" "./kit/BRD4104A/bsp_bcc.o" "./kit/BRD4104A/bsp_stk.o" "./kit/BRD4104A/bsp_stk_leds.o" "./kit/BRD4104A/i2cspm.o" "./kit/BRD4104A/si7013.o" "./kit/BRD4104A/tempsens.o" "./kit/BRD4104A/udelay.o" "./kit_flashpwr/BRD4104A/mx25flash_spi.o" "./kit_lcd/BRD4104A/display.o" "./kit_lcd/BRD4104A/displayls013b7dh03.o" "./kit_lcd/BRD4104A/displaypalemlib.o" "./serial_vcom/retargetio.o" "./serial_vcom/retargetserial.o" "./src/InitDevice.o" "./stack/bt_mesh_stack/mesh_lib.o" -lm -lbluetooth_mesh -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: btmesh-LightsModule.hex'
	arm-none-eabi-objcopy -O ihex "btmesh-LightsModule.axf" "btmesh-LightsModule.hex"
	@echo ' '

	@echo 'Building bin file: btmesh-LightsModule.bin'
	arm-none-eabi-objcopy -O binary "btmesh-LightsModule.axf" "btmesh-LightsModule.bin"
	@echo ' '

	@echo 'Building s37 file: btmesh-LightsModule.s37'
	arm-none-eabi-objcopy -O srec "btmesh-LightsModule.axf" "btmesh-LightsModule.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "btmesh-LightsModule.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) btmesh-LightsModule.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
