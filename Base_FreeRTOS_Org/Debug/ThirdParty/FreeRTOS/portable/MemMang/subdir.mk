################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/portable/MemMang/heap_4.c 

OBJS += \
./ThirdParty/FreeRTOS/portable/MemMang/heap_4.o 

C_DEPS += \
./ThirdParty/FreeRTOS/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/portable/MemMang/%.o ThirdParty/FreeRTOS/portable/MemMang/%.su: ../ThirdParty/FreeRTOS/portable/MemMang/%.c ThirdParty/FreeRTOS/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=1 -c -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/FreeRTOS/include" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/Config" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/OS" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/ThirdParty/SEGGER/SEGGER" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/PeripheralDrivers/Inc" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/Robotica/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/Robotica/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Base_FreeRTOS_Org/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang

clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang:
	-$(RM) ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.d ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.o ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.su

.PHONY: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang

