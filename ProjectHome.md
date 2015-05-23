Input and control system for a powered wheelchair using the ATxmega64A1 rev H.

If you want to become a collaborator, please ask.

To compile the code, use Atmel Studio 6.0.1996 - Service Pack 2 with AVRGCC 3.4.1.830.
In addition to programming the flash and eeprom files, the ATxmega64A1 brown-out detection (BOD) fuse bits need to be programmed to 2.9V or 2.7V, and BOD enabled continuously in order to avoid EEPROM corruption. The BOD fuse bits are not in the location described in the XMEGA A manual - look at the errata. If the target microcontroller ever changes to ATxmega64A1 rev G, the code may require changes (specifically code dealing with EEPROM).

To edit schematics use Eagle 5.11 (NOT Eagle 6)

SparkFun eagle 5.11 library: https://github.com/sparkfun/SparkFun-Eagle-Library

ads4: http://www.andrewsterian.com/courses/214/ads4.lbr

InductiveJoysticks



Quick joystick reference:
  * x-axis = horizontal = right/left = direction = blue wire
  * y-axis = vertical = fwd/rev = speed = yellow wire

Read about a problem with the LCD going blank [LCDGoesBlank](http://code.google.com/p/pwct/wiki/LCDGoesBlank)