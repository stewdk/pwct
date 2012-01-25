################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
wheelchair.lss \

FLASH_IMAGE += \
wheelchair.hex \

EEPROM_IMAGE += \
wheelchair.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: wheelchair.elf secondary-outputs

# Tool invocations
wheelchair.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,wheelchair.map -mmcu=atxmega64a1 -o "wheelchair.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

wheelchair.lss: wheelchair.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S wheelchair.elf  >"wheelchair.lss"
	@echo 'Finished building: $@'
	@echo ' '

wheelchair.hex: wheelchair.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex wheelchair.elf  "wheelchair.hex"
	@echo 'Finished building: $@'
	@echo ' '

wheelchair.eep: wheelchair.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex wheelchair.elf  "wheelchair.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: wheelchair.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atxmega64a1 wheelchair.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) wheelchair.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
