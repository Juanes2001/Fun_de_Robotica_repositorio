################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/syscalls.c 

OBJS += \
./Src/main.o \
./Src/syscalls.o 

C_DEPS += \
./Src/main.d \
./Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=1 -c -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/ThirdParty/FreeRTOS/include" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/ThirdParty/SEGGER/Config" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/ThirdParty/SEGGER/OS" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/ThirdParty/SEGGER/SEGGER" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/PeripheralDrivers/Inc" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/GitHub/Fun_de_Robotica_repositorio/Prueba_Astar/Inc" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/Robotica/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/juane/OneDrive/Desktop/STM_IDE/Robotica/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su

.PHONY: clean-Src

