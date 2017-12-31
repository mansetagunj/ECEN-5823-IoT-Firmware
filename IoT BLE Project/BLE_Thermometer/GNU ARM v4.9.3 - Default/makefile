################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include kit_flashpwr/BRD4302A/subdir.mk
-include kit/BRD4302A/subdir.mk
-include emlib/subdir.mk
-include device/EFR32_B_1_1_B/gcc/subdir.mk
-include bgapi/subdir.mk
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
all: BLE_Thermometer.axf

# Tool invocations
BLE_Thermometer.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\BLE_Thermometer\linker\GCC\efr32bg1b232f256gm56.ld" -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="BLE_Thermometer.map" -mfpu=fpv4-sp-d16 -mfloat-abi=hard -o BLE_Thermometer.axf "./application_properties.o" "./gatt_db.o" "./main.o" "./bgapi/dmadrv.o" "./bgapi/em_adc.o" "./bgapi/em_ldma.o" "./bgapi/em_leuart.o" "./device/EFR32_B_1_1_B/gcc/startup_efr32bg1b.o" "./emlib/em_cmu.o" "./emlib/em_cryotimer.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_letimer.o" "./emlib/em_msc.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./kit/BRD4302A/bsp_bcc.o" "./kit/BRD4302A/bsp_stk.o" "./kit/BRD4302A/bsp_stk_leds.o" "./kit/BRD4302A/i2cspm.o" "./kit/BRD4302A/si7013.o" "./kit/BRD4302A/tempsens.o" "./kit/BRD4302A/udelay.o" "./kit_flashpwr/BRD4302A/mx25flash_spi.o" "./src/I2C_TempSensor.o" "./src/InitDevice.o" "./src/adc0.o" "./src/cmu.o" "./src/gpio.o" "./src/letimer0.o" "./src/sleep_g.o" "./src/spi.o" "./src/timer0.o" "C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/GCC/bgapi.a" "C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binbootloader.o" "C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/stack.a" "C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binstack.o" -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: BLE_Thermometer.hex'
	arm-none-eabi-objcopy -O ihex "BLE_Thermometer.axf" "BLE_Thermometer.hex"
	@echo ' '

	@echo 'Building bin file: BLE_Thermometer.bin'
	arm-none-eabi-objcopy -O binary "BLE_Thermometer.axf" "BLE_Thermometer.bin"
	@echo ' '

	@echo 'Building s37 file: BLE_Thermometer.s37'
	arm-none-eabi-objcopy -O srec "BLE_Thermometer.axf" "BLE_Thermometer.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "BLE_Thermometer.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) BLE_Thermometer.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
