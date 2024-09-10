################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c 

OBJS += \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o 

C_DEPS += \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/SEGGER/Syscalls/%.o ThirdParty/SEGGER/SEGGER/Syscalls/%.su: ../ThirdParty/SEGGER/SEGGER/Syscalls/%.c ThirdParty/SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=1 -c -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/ThirdParty/SEGGER/Config" -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/ThirdParty/SEGGER/OS" -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/ThirdParty/SEGGER/SEGGER" -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/Base_FreeRTOS/Inc" -I/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include -I"/home/juan/FundamentosDeRobotica/Fun_de_Robotica_repositorio/PeripheralDriver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls

clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls

