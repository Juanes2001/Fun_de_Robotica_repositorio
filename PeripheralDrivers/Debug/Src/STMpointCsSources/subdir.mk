################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/STMpointCsSources/system_stm32f4xx.c 

OBJS += \
./Src/STMpointCsSources/system_stm32f4xx.o 

C_DEPS += \
./Src/STMpointCsSources/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/STMpointCsSources/%.o Src/STMpointCsSources/%.su Src/STMpointCsSources/%.cyclo: ../Src/STMpointCsSources/%.c Src/STMpointCsSources/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"D:/Workspace_STM/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Workspace_STM/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include" -I"C:/Users/Juanes/Desktop/GitHub/Fun_de_Robotica_repositorio/PeripheralDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-STMpointCsSources

clean-Src-2f-STMpointCsSources:
	-$(RM) ./Src/STMpointCsSources/system_stm32f4xx.cyclo ./Src/STMpointCsSources/system_stm32f4xx.d ./Src/STMpointCsSources/system_stm32f4xx.o ./Src/STMpointCsSources/system_stm32f4xx.su

.PHONY: clean-Src-2f-STMpointCsSources

