# Build the firmware
cargo build --release

# The ELF is at:
#   target/avr-unknown-gnu-atmega328/release/ctcss-gen.elf

# Use avr-size to inspect .data (initialised RAM) + .bss (zero-init RAM):
avr-size -C --mcu=atmega328p target/avr-none/release/ctcss-gen.elf
