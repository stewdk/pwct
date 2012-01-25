################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/nordic_driver.c \
../src/nordic_hardware_specific.c \
../src/remote_hardware.c 

OBJS += \
./src/main.o \
./src/nordic_driver.o \
./src/nordic_hardware_specific.o \
./src/remote_hardware.o 

C_DEPS += \
./src/main.d \
./src/nordic_driver.d \
./src/nordic_hardware_specific.d \
./src/remote_hardware.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=attiny261 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


