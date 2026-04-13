#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// ──────────────────────────────────────────────
//  CTCSS Tone Generator for Yaesu FT-480R
//
//  Hardware:
//    MCU       — ATmega328P (bare chip, ICSP programmed)
//    Display   — 128×64 SSD1315 OLED over SPI
//    Encoder   — rotary encoder on PD2/PD3 (INT0/INT1)
//    Buttons   — "Select" on PD4,  "Toggle output" on PD7
//    Tone out  — OC1A  (PB1 / Arduino-D9) via Timer1 CTC toggle
//    Power     — AA batteries through a power switch
//
//  Tone output (0 V – 5 V square from OC1A) must be attenuated
//  to ≤ 0.35 Vpp with a resistive divider before injecting into
//  the FT-480R tone socket.
// ──────────────────────────────────────────────

use atmega_hal::delay::Delay;
use atmega_hal::prelude::*;
use avr_device::atmega328p::TC1;
use core::cell;
use core::sync::atomic::{AtomicBool, Ordering};
use panic_halt as _;

use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1315::Ssd1315;
use ssd1315::interface::SpiDisplayInterface;

// ── Standard CTCSS tone table (EIA/TIA) ──────────────────────
/// All 50 standard CTCSS tones in deci-Hertz (671 = 67.1 Hz).
/// Stored in PROGMEM-friendly const; each is a u16 → 100 bytes.
const CTCSS_TONES: [u16; 50] = [
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974, 1000, 1035, 1072, 1109, 1148, 1188,
    1230, 1273, 1318, 1365, 1413, 1462, 1514, 1567, 1622, 1679, 1738, 1799, 1862, 1928, 1966, 2035,
    2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541, 2591, 2642, 2693, 2744, 2797, 2851, 2999,
];

// ── Shared state (interrupt ↔ main loop) ─────────────────────
/// Encoder direction: +1 or -1 per detent
static ENCODER_DELTA: avr_device::interrupt::Mutex<cell::Cell<i8>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

/// Button press flags
static BTN_SELECT_PRESSED: AtomicBool = AtomicBool::new(false);
static BTN_TOGGLE_PRESSED: AtomicBool = AtomicBool::new(false);

/// Millis counter driven by TC0
static MILLIS_CTR: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

// ── Helpers ──────────────────────────────────────────────────

fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_CTR.borrow(cs).get())
}

/// Read and reset the encoder delta accumulated since last call.
fn take_encoder_delta() -> i8 {
    avr_device::interrupt::free(|cs| {
        let c = ENCODER_DELTA.borrow(cs);
        let v = c.get();
        c.set(0);
        v
    })
}

// ─────────────────────────────────────────────────────────────
//  Timer 1 — CTC mode with OC1A toggle → square wave on PB1
// ─────────────────────────────────────────────────────────────
//
//  f_out = f_clk / (2 × prescaler × (1 + OCR1A))
//
//  Rearranged:
//      OCR1A = f_clk / (2 × prescaler × f_out) − 1
//
//  With prescaler = 8:
//      range: f_out_min ≈ 16 MHz / (2×8×65536) ≈ 15.3 Hz
//             f_out_max ≈ 16 MHz / (2×8×1)     = 1 MHz
//      so 50 Hz – 300 Hz CTCSS tones are comfortably in range.
//
//  Below is the key line for setting the output frequency.
//  You can copy/paste this into your own code or adjust the
//  prescaler by consulting Section 15.11 of the datasheet.
// ─────────────────────────────────────────────────────────────

/// CPU clock frequency.  Change to 8_000_000 if using the
/// internal 8 MHz RC oscillator (no crystal).
const F_CPU: u32 = 16_000_000;
const PRESCALER: u32 = 8;

/// Compute the OCR1A value for a target frequency given in
/// **deci-hertz** (tenths of a Hz).  E.g. 1000 → 100.0 Hz.
///
/// Uses integer arithmetic: OCR1A = (F_CPU * 10) / (2 * PRESCALER * freq_dhz) − 1
const fn ocr_for_dhz(freq_dhz: u32) -> u16 {
    let top = F_CPU * 10 / (2 * PRESCALER * freq_dhz) - 1;
    top as u16
}

/// Apply a new tone frequency to Timer 1.
/// `freq_dhz` is in deci-hertz (671 → 67.1 Hz).
fn set_tone_frequency(tc1: &TC1, freq_dhz: u16) {
    let ocr = ocr_for_dhz(freq_dhz as u32);

    // ──────────────────────────────────────────────────────
    //  THIS is the line that programs the output frequency.
    //  Datasheet §15.11.  CTC mode (WGM1 = 0b0100),
    //  toggle OC1A on compare match (COM1A = 0b01),
    //  prescaler /8 (CS1 = 0b010).
    //
    //  f_out = F_CPU / (2 × 8 × (1 + OCR1A))
    // ──────────────────────────────────────────────────────
    tc1.tccr1a()
        .write(|w| w.com1a().match_toggle().wgm1().set(0b00));
    tc1.tccr1b()
        .write(|w| w.wgm1().set(0b01).cs1().prescale_8());
    tc1.ocr1a().write(|w| w.set(ocr));
    // Reset counter so the new period starts cleanly
    tc1.tcnt1().write(|w| w.set(0));
}

/// Disable tone output (stop the timer, drive OC1A low).
fn stop_tone(tc1: &TC1) {
    tc1.tccr1b().write(|w| w.cs1().no_clock()); // stop clock
    tc1.tccr1a().write(|w| w.com1a().disconnected()); // release pin
}

// ── Millis timer (TC0, CTC, ~1 ms) ──────────────────────────
fn millis_init(tc0: atmega_hal::pac::TC0) {
    // 16 MHz / 64 / 250 = 1 kHz → 1 ms per tick
    tc0.tccr0a().write(|w| w.wgm0().ctc());
    tc0.ocr0a().write(|w| w.set(249u8)); // 250 counts
    tc0.tccr0b().write(|w| w.cs0().prescale_64());
    tc0.timsk0().write(|w| w.ocie0a().set_bit());
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let c = MILLIS_CTR.borrow(cs);
        c.set(c.get().wrapping_add(1));
    });
}

// ── Rotary encoder ISR (on PCINT2 — PORTD) ──────────────────
// Encoder A → PD2, Encoder B → PD3.
// We use pin-change interrupt group 2 (PCINT16..23 = PORTD).
// A simple "read both pins on any edge" grey-code decode.

static mut LAST_ENC: u8 = 0;

/// Call once to set up PCINT2 on PD2 and PD3.
fn encoder_init(exint: &atmega_hal::pac::EXINT) {
    // Enable PCINT group 2 (PORTD pins)
    exint.pcicr().write(|w| unsafe { w.bits(0b100) });
    // Enable PCINT18 (PD2) and PCINT19 (PD3)
    exint.pcmsk2().write(|w| unsafe { w.bits(0b0000_1100) });
}

#[avr_device::interrupt(atmega328p)]
fn PCINT2() {
    // Read PD2 (bit 2) and PD3 (bit 3) from PIND
    let pind = unsafe { (*avr_device::atmega328p::PORTD::ptr()).pin().read().bits() };
    let a = (pind >> 2) & 1;
    let b = (pind >> 3) & 1;
    let enc = (a << 1) | b;

    let last = unsafe { LAST_ENC };
    // Simple grey-code transition table
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

    if delta != 0 {
        avr_device::interrupt::free(|cs| {
            let c = ENCODER_DELTA.borrow(cs);
            c.set(c.get().saturating_add(delta));
        });
    }
    unsafe {
        LAST_ENC = enc;
    }
}

// ── Display helpers ──────────────────────────────────────────

/// Format a deci-hertz value like 1318 into "131.8" inside `buf`.
/// Returns the number of bytes written (always 5 for 3-digit +
/// decimal, or 6 for 4-digit values, etc.).
fn fmt_dhz(buf: &mut [u8; 8], dhz: u16) -> usize {
    let whole = dhz / 10;
    let frac = dhz % 10;

    // Simple manual decimal formatting (no alloc)
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
        // reverse
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

// ── Entry point ──────────────────────────────────────────────
#[avr_device::entry]
fn main() -> ! {
    let dp = atmega_hal::Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);

    // ── SPI bus for OLED ─────────────────────────────────────
    // SCK  = PB5 (D13)
    // MOSI = PB3 (D11)
    // CS   = PB2 (D10) — directly wired to display CS
    // DC   = PB0 (D8)  — data/command select
    // RST  = PC0 (A0)  — optional, directly-wired reset
    let spi_settings = atmega_hal::spi::Settings {
        data_order: atmega_hal::spi::DataOrder::MostSignificantFirst,
        clock: atmega_hal::spi::SerialClockRate::OscfOver4,
        mode: embedded_hal::spi::MODE_0,
    };
    let (spi, _) = atmega_hal::Spi::new(
        dp.SPI,
        pins.pb5.into_output(),        // SCK
        pins.pb3.into_output(),        // MOSI
        pins.pb4.into_pull_up_input(), // MISO (unused, but SPI periph needs it)
        pins.pb2.into_output(),        // CS
        spi_settings,
    );

    let dc = pins.pb0.into_output(); // Data/Command
    let mut rst = pins.pc0.into_output(); // Reset (directly-driven)

    // Hard-reset the display
    rst.set_low();
    delay_ms(10);
    rst.set_high();
    delay_ms(10);

    // Build the display-interface SPI adapter, then the ssd1315 driver
    let spi_iface = SpiDisplayInterface::new_interface(spi, dc);
    let mut display = Ssd1315::new(spi_iface);
    display.init_screen();

    // ── Tone output pin ──────────────────────────────────────
    // OC1A = PB1 = Arduino D9.  Must be set as output so the
    // timer compare-match can toggle it.
    let _tone_pin = pins.pb1.into_output();

    // ── Encoder + buttons ────────────────────────────────────
    let _enc_a = pins.pd2.into_pull_up_input(); // PD2 — encoder A
    let _enc_b = pins.pd3.into_pull_up_input(); // PD3 — encoder B
    let btn_sel = pins.pd4.into_pull_up_input(); // PD4 — "select digit"
    let btn_tog = pins.pd7.into_pull_up_input(); // PD7 — "toggle output"

    // ── Timers ───────────────────────────────────────────────
    millis_init(dp.TC0);
    encoder_init(&dp.EXINT);

    let tc1 = dp.TC1;

    // Enable interrupts globally
    unsafe {
        avr_device::interrupt::enable();
    }

    // ── Application state ────────────────────────────────────
    let mut tone_index: usize = 12; // start at 100.0 Hz (index 12)
    let mut output_on = false;
    let mut last_btn_sel = true; // pull-up → idle high
    let mut last_btn_tog = true;
    let mut last_render: u32 = 0;
    let mut needs_redraw = true;

    // Initial tone load (but don't start output yet)
    stop_tone(&tc1);

    // ── Main loop ────────────────────────────────────────────
    loop {
        let now = millis();

        // -- Encoder: scroll through the CTCSS table ----------
        let delta = take_encoder_delta();
        if delta != 0 {
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

        // -- Button: select (currently unused placeholder) ----
        let sel_now = btn_sel.is_high();
        if last_btn_sel && !sel_now {
            // Falling edge — button pressed (active-low)
            BTN_SELECT_PRESSED.store(true, Ordering::SeqCst);
        }
        last_btn_sel = sel_now;

        // -- Button: toggle tone output on/off ----------------
        let tog_now = btn_tog.is_high();
        if last_btn_tog && !tog_now {
            output_on = !output_on;
            if output_on {
                set_tone_frequency(&tc1, CTCSS_TONES[tone_index]);
            } else {
                stop_tone(&tc1);
            }
            needs_redraw = true;
        }
        last_btn_tog = tog_now;

        // -- Redraw display at most every ~80 ms ──────────────
        if needs_redraw && now.wrapping_sub(last_render) >= 80 {
            needs_redraw = false;
            last_render = now;

            // Format frequency string
            let mut fbuf = [0u8; 8];
            let flen = fmt_dhz(&mut fbuf, CTCSS_TONES[tone_index]);
            let freq_str = core::str::from_utf8(&fbuf[..flen]).unwrap_or("???");

            let big_style = MonoTextStyleBuilder::new()
                .font(&FONT_10X20)
                .text_color(BinaryColor::On)
                .build();
            let small_style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .build();

            // Clear and draw
            display.clear(BinaryColor::Off).ok();

            Text::with_baseline("CTCSS Tone", Point::new(16, 0), small_style, Baseline::Top)
                .draw(&mut display)
                .ok();

            Text::with_baseline(freq_str, Point::new(14, 24), big_style, Baseline::Top)
                .draw(&mut display)
                .ok();

            let status = if output_on {
                ">> OUTPUT ON <<"
            } else {
                "   output off  "
            };
            Text::with_baseline(status, Point::new(10, 52), small_style, Baseline::Top)
                .draw(&mut display)
                .ok();

            display.flush_screen();
        }

        // Small delay to debounce
        delay_ms(2);
    }
}
