# CTCSS Generator

Generates a user-configurable tone for input to a radio without built-in CTCSS tone generation.

CTCSS stands for continuous tone-coded squelch system. Amateur radio repeaters often receive on one frequency
and transmit/repeat on another. In order to avoid the need for continuous operation, they only activate after
receiving a specific tone. These tones are typically between 80Hz and 2kHz using the same modulation as voice.

Initially designed for my Yaesu FT-480R, a 2m transciever from the 1980s before CTCSS repeater operation was common.
The transciever has a port where a 0.35V tone signal can be supplied. This is injected after the microphone
pre-amplifier for transmission.
