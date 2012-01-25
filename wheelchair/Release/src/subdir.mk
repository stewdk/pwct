################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/PWCT_io.c \
../src/TC_driver.c \
../src/adc_driver.c \
../src/bumper.c \
../src/clksys_driver.c \
../src/linear_actuator.c \
../src/main.c \
../src/nordic_driver.c \
../src/nordic_hardware_specific.c \
../src/pmic_driver.c \
../src/port_driver.c \
../src/spi_driver.c \
../src/usart_driver.c \
../src/util.c \
../src/wdt_driver.c 

OBJS += \
./src/PWCT_io.o \
./src/TC_driver.o \
./src/adc_driver.o \
./src/bumper.o \
./src/clksys_driver.o \
./src/linear_actuator.o \
./src/main.o \
./src/nordic_driver.o \
./src/nordic_hardware_specific.o \
./src/pmic_driver.o \
./src/port_driver.o \
./src/spi_driver.o \
./src/usart_driver.o \
./src/util.o \
./src/wdt_driver.o 

C_DEPS += \
./src/PWCT_io.d \
./src/TC_driver.d \
./src/adc_driver.d \
./src/bumper.d \
./src/clksys_driver.d \
./src/linear_actuator.d \
./src/main.d \
./src/nordic_driver.d \
./src/nordic_hardware_specific.d \
./src/pmic_driver.d \
./src/port_driver.d \
./src/spi_driver.d \
./src/usart_driver.d \
./src/util.d \
./src/wdt_driver.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atxmega64a1 -DF_CPU=32000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


