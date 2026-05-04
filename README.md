# CTCSS Generator

Generates a user-configurable tone for input to a radio without built-in CTCSS tone generation.

CTCSS stands for continuous tone-coded squelch system. Amateur radio repeaters often receive on one frequency
and transmit/repeat on another. In order to avoid the need for continuous operation, they only activate after
receiving a specific tone. These tones are typically between 80Hz and 2kHz using the same modulation as voice.

Initially designed for my Yaesu FT-480R, a 2m transciever from the 1980s before CTCSS repeater operation was common.
The transciever has a port where a 0.35V tone signal can be supplied. This is injected after the microphone
pre-amplifier for transmission.

## Programming

To program with avrdude first convert the built `.elf` to a `.hex` file
```
avr-objcopy -O ihex target/avr-none/release/ctcss-gen.elf target/avr-none/release/ctcss-gen.hex
```

Then upload the program
```
avrdude -c stk500v2 -P /dev/ttyACM0 -p m328p -U flash:w:target/avr-none/release/ctcss-gen.hex
```

The fuse bits should be set too, though this only needs to be done once
```
avrdude -c stk500v2 -P /dev/ttyACM0 -p m328p -U lfuse:w:0xFF:m   -U hfuse:w:0xD9:m   -U efuse:w:0xFD:m
```
