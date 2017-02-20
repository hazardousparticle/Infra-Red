# Infra-Red
Infra Red Remote wireless replay 

A basic implementation of the IR protocol used by Panasonic TV remotes. 
For ATMEGA32 AVR microprocessors.

Compile with avr-gcc

generate hex with:
avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex $(TARGET_OUTPUT_FILE) $(TARGET_OUTPUT_DIR)$(TARGET_OUTPUT_BASENAME).hex

Flash with:
avrdude -p atmega32 -P usb -c usbasp -U flash:w:$(TARGET_OUTPUT_BASENAME).hex

Connect an infra-red LED to the PWM on port PD5 of the AVR.
It is recommended to connect the LED through a transistor to prevent overcurent on the microcontroller.
