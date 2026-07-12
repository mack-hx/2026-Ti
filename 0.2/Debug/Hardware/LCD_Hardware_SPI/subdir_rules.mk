################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Hardware/LCD_Hardware_SPI/%.o: ../Hardware/LCD_Hardware_SPI/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"/Users/m/ti/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.2" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.2/Debug" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.2/Hardware/LCD_Hardware_SPI" -I"/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI/0.2/Hardware/PD42S1" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/m/ti/mspm0_sdk_2_10_00_04/source" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"/Users/m/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"Hardware/LCD_Hardware_SPI/$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


