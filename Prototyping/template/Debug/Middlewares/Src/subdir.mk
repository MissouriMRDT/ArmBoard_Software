################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Src/PRJ_main.c 

OBJS += \
./Middlewares/Src/PRJ_main.o 

C_DEPS += \
./Middlewares/Src/PRJ_main.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Src/PRJ_main.o: ../Middlewares/Src/PRJ_main.c Middlewares/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H563xx -c -I../Core/Inc -I../Middlewares/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Src

clean-Middlewares-2f-Src:
	-$(RM) ./Middlewares/Src/PRJ_main.cyclo ./Middlewares/Src/PRJ_main.d ./Middlewares/Src/PRJ_main.o ./Middlewares/Src/PRJ_main.su

.PHONY: clean-Middlewares-2f-Src

