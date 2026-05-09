#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//  CTCSS Tone Generator for Yaesu FT-480R
//
//  Hardware:
//    MCU       — ATmega328P (bare chip, ICSP programmed)
//    Display   — 128×64 SSD1306 OLED over I2C (addr 0x3C)
//    Encoder   — rotary encoder on PD2/PD3 (PCINT18/19)
//    Buttons   — "Select" on PD4
//    Tone out  — OC1A  (PB1) via Timer1 CTC toggle
//    Power     — AA batteries through a power switch
//
//  I2C pins (ATmega328P TWI):
//    SDA = PC4
//    SCL = PC5
//
//  Timers:
//  TC0 = millisecond counter
//  TC1 = tone generator
//
//  Tone output (0 V – 3.3 V square from OC1A) must be attenuated
//  to ≤ 0.35 Vpp with a resistive divider before injecting into
//  the FT-480R tone socket.
//
//  Each time a PortD input pin (i.e. from the rotary encoder)
//  changes state, the PCINT2 interrupt is triggered. Here, the
//  state of the rotary encoder is compared to previous to
//  determine direction of rotation. If this input is after the
//  debounce cooldown period, it is recorded. In the next main
//  loop iteration, it changes the selected CTCSS tone.

use core::cell::Cell;
use core::fmt::Write;
use panic_halt as _;

use atmega_hal::usart::Baudrate;
use atmega_hal::usart::Usart;
use avr_device::atmega328p::TC1;
use avr_device::interrupt::Mutex;

use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

// Standard CTCSS tone table (EIA/TIA)
const CTCSS_TONES: [u16; 50] = [
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974, 1000, 1035, 1072, 1109, 1148, 1188,
    1230, 1273, 1318, 1365, 1413, 1462, 1514, 1567, 1622, 1679, 1738, 1799, 1862, 1928, 1966, 2035,
    2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541, 2591, 2642, 2693, 2744, 2797, 2851, 2999,
];

// Shared state (interrupts & main loop)
static ENCODER_DELTA: Mutex<Cell<i8>> = Mutex::new(Cell::new(0));

static MILLIS_CTR: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static DEBOUNCE_CTR: Mutex<Cell<u32>> = Mutex::new(Cell::new(0)); // Time of last input
const DEBOUNCE_COOLDOWN: u32 = 250; // Cooldown for switch inputs, in milliseconds

// Helper functions

/// The current calue of a millisecond counter
fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_CTR.borrow(cs).get())
}

/// Check whether cooldown timer has elapsed. If so, resets the counter
fn cooled_down() -> bool {
    avr_device::interrupt::free(|cs| {
        let now = MILLIS_CTR.borrow(cs).get();
        let prev_time_cell = DEBOUNCE_CTR.borrow(cs);
        let time_elapsed = now.wrapping_sub(prev_time_cell.get());
        if time_elapsed > DEBOUNCE_COOLDOWN {
            prev_time_cell.set(now);
            true
        } else {
            false
        }
    })
}

fn take_encoder_delta() -> i8 {
    avr_device::interrupt::free(|cs| {
        let c = ENCODER_DELTA.borrow(cs);
        let v = c.get();
        c.set(0);
        v
    })
}

// Timer 1 — tone generation

const F_CPU: u32 = 16_000_000;
const PRESCALER: u32 = 8;

const fn ocr_for_dhz(freq_dhz: u32) -> u16 {
    let top = F_CPU * 10 / (2 * PRESCALER * freq_dhz) - 1;
    top as u16
}

/// Configures TC1 peripheral
fn set_tone_frequency(tc1: &TC1, freq_dhz: u16) {
    // Sets to CTC mode (WGM1 bits to 0100)
    // Sets OC1A pin to toggle on compare match
    // Sets clock source to CLK-IO/8
    // Sets OCR1A to the calculated value
    // Sets the timer value to zero
    let ocr = ocr_for_dhz(freq_dhz as u32);
    tc1.tccr1a()
        .write(|w| w.com1a().match_toggle().wgm1().set(0b00));
    tc1.tccr1b()
        .write(|w| w.wgm1().set(0b01).cs1().prescale_8());
    tc1.ocr1a().write(|w| w.set(ocr));
    tc1.tcnt1().write(|w| w.set(0));
}

/// Stops the TC1 peripheral
fn stop_tone(tc1: &TC1) {
    // Stops the clock source and disconnects OC1A pin
    tc1.tccr1b().write(|w| w.cs1().no_clock());
    tc1.tccr1a().write(|w| w.com1a().disconnected());
}

/// Initialises millis on timer 0
fn millis_init(tc0: atmega_hal::pac::TC0) {
    // Sets CTC (clear timer on compare) mode
    // Sets the output compare register (OCR0A)
    // Sets the clock source to CLK-IO/64
    // Enables interrupt on output-compare match
    tc0.tccr0a().write(|w| w.wgm0().ctc());
    tc0.ocr0a().write(|w| w.set(249u8));
    tc0.tccr0b().write(|w| w.cs0().prescale_64());
    tc0.timsk0().write(|w| w.ocie0a().set_bit());
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    // Timer 0 Compare A interrupt
    // Updates the millisecond counter
    avr_device::interrupt::free(|cs| {
        let c = MILLIS_CTR.borrow(cs);
        c.set(c.get().wrapping_add(1));
    });
}

// Rotary encoder ISR

static mut LAST_ENC: u8 = 0;

fn encoder_init(exint: &atmega_hal::pac::EXINT) {
    // Sets pin change interrupt 2 enable
    exint.pcicr().write(|w| unsafe { w.bits(0b100) });
    exint.pcmsk2().write(|w| unsafe { w.bits(0b0000_1100) });
}

#[avr_device::interrupt(atmega328p)]
fn PCINT2() {
    // Pin change 2 interrupt
    // Read state of rotary encoder pins
    let pind = unsafe { (*atmega_hal::pac::PORTD::ptr()).pind().read().bits() };
    let a = (pind >> 2) & 1;
    let b = (pind >> 3) & 1;
    let enc = (a << 1) | b;

    let last = unsafe { LAST_ENC };
    let delta: i8 = match (last, enc) {
        (0b00, 0b01) => 1,
        (0b01, 0b11) => 1,
        (0b11, 0b10) => 1,
        (0b10, 0b00) => 1,
        (0b00, 0b10) => -1,
        (0b10, 0b11) => -1,
        (0b11, 0b01) => -1,
        (0b01, 0b00) => -1,
        _ => 0,
    };

    if delta != 0 && cooled_down() {
        avr_device::interrupt::free(|cs| {
            let c = ENCODER_DELTA.borrow(cs);
            //c.set(c.get().saturating_add(delta));
            c.set(delta);
        });
    }
    unsafe {
        LAST_ENC = enc;
    }
}

// Display helper
fn fmt_dhz(buf: &mut [u8; 8], dhz: u16) -> usize {
    let whole = dhz / 10;
    let frac = dhz % 10;

    let mut pos = 0;
    let mut tmp = [0u8; 5];
    let mut w = whole;
    if w == 0 {
        tmp[0] = b'0';
        pos = 1;
    } else {
        while w > 0 {
            tmp[pos] = b'0' + (w % 10) as u8;
            w /= 10;
            pos += 1;
        }
        tmp[..pos].reverse();
    }
    let mut out = 0;
    for i in 0..pos {
        buf[out] = tmp[i];
        out += 1;
    }
    buf[out] = b'.';
    out += 1;
    buf[out] = b'0' + frac as u8;
    out += 1;
    buf[out] = b' ';
    out += 1;
    buf[out] = b'H';
    out += 1;
    buf[out] = b'z';
    out += 1;
    out
}

// Clock type for I2C speed calculation
// TODO: Check clock speed
type CoreClock = atmega_hal::clock::MHz16;
type I2c = atmega_hal::i2c::I2c<CoreClock>;

// Entry point
#[avr_device::entry]
fn main() -> ! {
    let dp = atmega_hal::Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);

    let mut serial = Usart::new(
        dp.USART0,
        pins.pd0,               // Rx
        pins.pd1.into_output(), // Tx
        Baudrate::<CoreClock>::new(115200),
    );

    ufmt::uwriteln!(&mut serial, "Booting").unwrap();

    // I2C bus for OLED
    //  SDA = PC4,  SCL = PC5  (ATmega328P TWI hardware)
    let mut i2c = I2c::new(
        dp.TWI,
        pins.pc4.into_pull_up_input(), // SDA
        pins.pc5.into_pull_up_input(), // SCL
        400_000,                       // 400 kHz Fast-mode
    );

    ufmt::uwriteln!(&mut serial, "I2C defined").unwrap();

    // Build I2C display interface → ssd1306 driver in terminal graphics mode
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    let r = display.init();
    ufmt::uwriteln!(&mut serial, "display.init={:?}", r.is_ok()).ok();
    let r = display.clear();
    ufmt::uwriteln!(&mut serial, "display.clear={:?}", r.is_ok()).ok();
    ufmt::uwriteln!(&mut serial, "display.dimensions={:?}", display.dimensions()).ok();
    display.set_position(0, 0).ok();
    for c in "STARTUP".chars() {
        // Writing strings doesn't work but individual chars is ok???
        display.write_char(c).unwrap();
    }
    ufmt::uwriteln!(&mut serial, "Display initialised").ok();

    // Tone output pin
    let _tone_pin = pins.pb1.into_output();

    // Encoder + buttons
    let _enc_a = pins.pd2.into_floating_input();
    let _enc_b = pins.pd3.into_floating_input();
    let btn_sel = pins.pd4.into_floating_input();

    // Timers setup
    millis_init(dp.TC0);
    encoder_init(&dp.EXINT);

    ufmt::uwriteln!(&mut serial, "Timers and encoder initialised").unwrap();

    let tc1 = dp.TC1;

    unsafe {
        avr_device::interrupt::enable();
    }

    ufmt::uwriteln!(&mut serial, "Interrupts enabled").unwrap();

    // Application state
    let mut tone_index: usize = 12;
    let mut output_on = false;
    let mut last_render: u32 = 0;
    let mut needs_redraw = true;

    stop_tone(&tc1);

    // Main loop
    loop {
        let now = millis();

        let delta = take_encoder_delta();
        if delta != 0 {
            ufmt::uwriteln!(&mut serial, "Updating tone").unwrap();
            let new_idx = (tone_index as i16 + delta as i16)
                .clamp(0, (CTCSS_TONES.len() - 1) as i16) as usize;
            if new_idx != tone_index {
                tone_index = new_idx;
                if output_on {
                    set_tone_frequency(&tc1, CTCSS_TONES[tone_index]);
                }
                needs_redraw = true;
            }
        }

        let sel_now = btn_sel.is_low() && cooled_down();
        if sel_now {
            ufmt::uwriteln!(&mut serial, "Toggle pressed").unwrap();
            output_on = !output_on;
            if output_on {
                set_tone_frequency(&tc1, CTCSS_TONES[tone_index]);
            } else {
                stop_tone(&tc1);
            }
            needs_redraw = true;
        }

        if needs_redraw && now.wrapping_sub(last_render) >= 80 {
            ufmt::uwriteln!(&mut serial, "REDRAWING").unwrap();
            let r = display.clear();
            ufmt::uwriteln!(&mut serial, "display.clear={:?}", r.is_ok()).ok();
            let r = display.set_position(4, 0);
            ufmt::uwriteln!(&mut serial, "display.set_position={:?}", r.is_ok()).ok();
            for c in "  CTCSS Tone  ".chars() {
                display.write_char(c).unwrap();
            }
            // write!(display, "  CTCSS Tone  ").unwrap();
            ufmt::uwriteln!(&mut serial, "Cleared & header written").unwrap();

            let mut fbuf = [0u8; 8];
            let flen = fmt_dhz(&mut fbuf, CTCSS_TONES[tone_index]);
            let freq_str = core::str::from_utf8(&fbuf[..flen]).unwrap_or("???");
            let r = display.set_position(3, 3);
            for c in freq_str.chars() {
                display.write_char(c).unwrap();
            }
            // write!(display, "{}", freq_str).unwrap();
            ufmt::uwriteln!(
                &mut serial,
                "Tone freq written, display.set_position={:?}",
                r.is_ok(),
            )
            .ok();

            // Row 6 – output status
            let status = if output_on {
                ">> OUTPUT ON <<"
            } else {
                "  output off   "
            };
            display.set_position(1, 6).unwrap();
            for c in status.chars() {
                display.write_char(c).unwrap();
            }
            // write!(display, "{}", status).unwrap();
            ufmt::uwriteln!(&mut serial, "Output status written").ok();

            needs_redraw = false;
            last_render = now;
            ufmt::uwriteln!(&mut serial, "REDRAW COMPLETE").ok();
        }
    }
}
