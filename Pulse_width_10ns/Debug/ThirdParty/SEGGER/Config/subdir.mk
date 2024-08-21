################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/Config/%.o ThirdParty/SEGGER/Config/%.su: ../ThirdParty/SEGGER/Config/%.c ThirdParty/SEGGER/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=1 -c -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/ThirdParty/FreeRTOS/include" -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/ThirdParty/SEGGER/Config" -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/ThirdParty/SEGGER/OS" -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/ThirdParty/SEGGER/SEGGER" -I"/home/juan/FundamentosDeRobotica/Pruebas_sin_FreeRTOS/Inc" -I/home/juan/FundamentosDeRobotica/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/home/juan/FundamentosDeRobotica/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include -I/home/juan/FundamentosDeRobotica/PeripheralDriver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-Config

clean-ThirdParty-2f-SEGGER-2f-Config:
	-$(RM) ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-Config

