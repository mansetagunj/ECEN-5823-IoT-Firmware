################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1/platform/Device/SiliconLabs/EFR32BG13P/Source/system_efr32bg13p.c 

OBJS += \
./device/EFR32_B_1_3_P/system_efr32bg13p.o 

C_DEPS += \
./device/EFR32_B_1_3_P/system_efr32bg13p.d 


# Each subdirectory must supply rules for building sources it contributes
device/EFR32_B_1_3_P/system_efr32bg13p.o: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1/platform/Device/SiliconLabs/EFR32BG13P/Source/system_efr32bg13p.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-D__STACK_SIZE=0xa00' '-D__HEAP_SIZE=0x1200' '-DRETARGET_VCOM=1' '-DSERIAL_ECHO=1' '-DSILABS_AF_USE_HWCONF=1' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\btmesh-DoorModule" -I"C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\btmesh-DoorModule\src" -I"C:\Users\Gunj Manseta\SimplicityStudio\v4_workspace\btmesh-DoorModule\inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/ble_mesh/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/include/common" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/include/soc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/Device/SiliconLabs/EFR32BG13P/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/dmadrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/sleep/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/uartdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/tempdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emdrv/tempdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/middleware/glib" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/dmd" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/dmd/ssd2119" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/glib" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//hardware/kit/EFR32BG13_BRD4104A/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//hardware/kit/EFR32MG13_BRD4104A/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/chip/efr32/rf/common/cortex" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/chip/efr32" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/blemesh/v1.1//app/bluetooth_dev/appbuilder/sample-apps/common/read_char" -O2 -fno-builtin -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"device/EFR32_B_1_3_P/system_efr32bg13p.d" -MT"device/EFR32_B_1_3_P/system_efr32bg13p.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


