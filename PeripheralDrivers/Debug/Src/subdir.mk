################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/AdcDriver.c \
../Src/BasicTimer.c \
../Src/CaptureFreqDriver.c \
../Src/DMA.c \
../Src/EXTIDriver.c \
../Src/FPUDriver.c \
../Src/GPIOxDriver.c \
../Src/I2CDriver.c \
../Src/MPUAccel.c \
../Src/MotorsDriver.c \
../Src/OLEDDriver.c \
../Src/PIDDriver.c \
../Src/PosRobt.c \
../Src/PwmDriver.c \
../Src/RCCHunMHz.c \
../Src/RTCDriver.c \
../Src/SPIDriver.c \
../Src/SysTickDriver.c \
../Src/USARTxDriver.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/AdcDriver.o \
./Src/BasicTimer.o \
./Src/CaptureFreqDriver.o \
./Src/DMA.o \
./Src/EXTIDriver.o \
./Src/FPUDriver.o \
./Src/GPIOxDriver.o \
./Src/I2CDriver.o \
./Src/MPUAccel.o \
./Src/MotorsDriver.o \
./Src/OLEDDriver.o \
./Src/PIDDriver.o \
./Src/PosRobt.o \
./Src/PwmDriver.o \
./Src/RCCHunMHz.o \
./Src/RTCDriver.o \
./Src/SPIDriver.o \
./Src/SysTickDriver.o \
./Src/USARTxDriver.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/AdcDriver.d \
./Src/BasicTimer.d \
./Src/CaptureFreqDriver.d \
./Src/DMA.d \
./Src/EXTIDriver.d \
./Src/FPUDriver.d \
./Src/GPIOxDriver.d \
./Src/I2CDriver.d \
./Src/MPUAccel.d \
./Src/MotorsDriver.d \
./Src/OLEDDriver.d \
./Src/PIDDriver.d \
./Src/PosRobt.d \
./Src/PwmDriver.d \
./Src/RCCHunMHz.d \
./Src/RTCDriver.d \
./Src/SPIDriver.d \
./Src/SysTickDriver.d \
./Src/USARTxDriver.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"D:/Workspace_STM/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Workspace_STM/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include" -I"C:/Users/Juanes/Desktop/GitHub/Fun_de_Robotica_repositorio/PeripheralDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/AdcDriver.cyclo ./Src/AdcDriver.d ./Src/AdcDriver.o ./Src/AdcDriver.su ./Src/BasicTimer.cyclo ./Src/BasicTimer.d ./Src/BasicTimer.o ./Src/BasicTimer.su ./Src/CaptureFreqDriver.cyclo ./Src/CaptureFreqDriver.d ./Src/CaptureFreqDriver.o ./Src/CaptureFreqDriver.su ./Src/DMA.cyclo ./Src/DMA.d ./Src/DMA.o ./Src/DMA.su ./Src/EXTIDriver.cyclo ./Src/EXTIDriver.d ./Src/EXTIDriver.o ./Src/EXTIDriver.su ./Src/FPUDriver.cyclo ./Src/FPUDriver.d ./Src/FPUDriver.o ./Src/FPUDriver.su ./Src/GPIOxDriver.cyclo ./Src/GPIOxDriver.d ./Src/GPIOxDriver.o ./Src/GPIOxDriver.su ./Src/I2CDriver.cyclo ./Src/I2CDriver.d ./Src/I2CDriver.o ./Src/I2CDriver.su ./Src/MPUAccel.cyclo ./Src/MPUAccel.d ./Src/MPUAccel.o ./Src/MPUAccel.su ./Src/MotorsDriver.cyclo ./Src/MotorsDriver.d ./Src/MotorsDriver.o ./Src/MotorsDriver.su ./Src/OLEDDriver.cyclo ./Src/OLEDDriver.d ./Src/OLEDDriver.o ./Src/OLEDDriver.su ./Src/PIDDriver.cyclo ./Src/PIDDriver.d ./Src/PIDDriver.o ./Src/PIDDriver.su ./Src/PosRobt.cyclo ./Src/PosRobt.d ./Src/PosRobt.o ./Src/PosRobt.su ./Src/PwmDriver.cyclo ./Src/PwmDriver.d ./Src/PwmDriver.o ./Src/PwmDriver.su ./Src/RCCHunMHz.cyclo ./Src/RCCHunMHz.d ./Src/RCCHunMHz.o ./Src/RCCHunMHz.su ./Src/RTCDriver.cyclo ./Src/RTCDriver.d ./Src/RTCDriver.o ./Src/RTCDriver.su ./Src/SPIDriver.cyclo ./Src/SPIDriver.d ./Src/SPIDriver.o ./Src/SPIDriver.su ./Src/SysTickDriver.cyclo ./Src/SysTickDriver.d ./Src/SysTickDriver.o ./Src/SysTickDriver.su ./Src/USARTxDriver.cyclo ./Src/USARTxDriver.d ./Src/USARTxDriver.o ./Src/USARTxDriver.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

