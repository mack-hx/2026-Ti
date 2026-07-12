################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-476156608: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"/Users/m/ti/sysconfig_1.26.2/sysconfig_cli.sh" -s "/Users/m/ti/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/empty.syscfg" -o "." --compiler gcc
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.lds: build-476156608 ../empty.syscfg
device.opt: build-476156608
device.lds.genlibs: build-476156608
ti_msp_dl_config.c: build-476156608
ti_msp_dl_config.h: build-476156608
Event.dot: build-476156608

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"/Users/m/ti/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Debug" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/LCD_Hardware_SPI" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/PD42S1" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_gcc.o: /Users/m/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"/Users/m/ti/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Debug" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/LCD_Hardware_SPI" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/PD42S1" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"/Users/m/ti/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Debug" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/LCD_Hardware_SPI" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.1/Hardware/PD42S1" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


