################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/%.o ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/%.su: ../ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/%.c ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=1 -c -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/FreeRTOS/include" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/Config" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/OS" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/SEGGER" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/Inc" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio/PeripheralDrivers/Inc" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio//STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Archivos, documentos Universidad/Fundamentos de robotica/Repositorio/Fun_de_Robotica_repositorio//STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F

clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.d ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.o ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F

