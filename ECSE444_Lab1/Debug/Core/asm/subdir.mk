################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/asm/kalman.s 

OBJS += \
./Core/asm/kalman.o 

S_DEPS += \
./Core/asm/kalman.d 


# Each subdirectory must supply rules for building sources it contributes
Core/asm/kalman.o: ../Core/asm/kalman.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/asm/kalman.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

