#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// ──────────────────────────────────────────────
//  CTCSS Tone Generator for Yaesu FT-480R
//
//  Hardware:
//    MCU       — ATmega328P (bare chip, ICSP programmed)
//    Display   — 128×64 SSD1306 OLED over I2C (addr 0x3C)
//    Encoder   — rotary encoder on PD2/PD3 (PCINT18/19)
//    Buttons   — "Select" on PD4,  "Toggle output" on PD7
//    Tone out  — OC1A  (PB1) via Timer1 CTC toggle
//    Power     — AA batteries through a power switch
//
//  I2C pins (ATmega328P TWI):
//    SDA = PC4
//    SCL = PC5
//
//  Tone output (0 V – 5 V square from OC1A) must be attenuated
//  to ≤ 0.35 Vpp with a resistive divider before injecting into
//  the FT-480R tone socket.
// ──────────��───────────────────────────────────

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

// ── Changed: ssd1306 I2C imports instead of ssd1315 SPI ──
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};

// ── Standard CTCSS tone table (EIA/TIA) ──────────────────────
const CTCSS_TONES: [u16; 50] = [
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974, 1000, 1035, 1072, 1109, 1148, 1188,
    1230, 1273, 1318, 1365, 1413, 1462, 1514, 1567, 1622, 1679, 1738, 1799, 1862, 1928, 1966, 2035,
    2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541, 2591, 2642, 2693, 2744, 2797, 2851, 2999,
];

// ── Shared state (interrupt ↔ main loop) ─────────────────────
static ENCODER_DELTA: avr_device::interrupt::Mutex<cell::Cell<i8>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

static BTN_SELECT_PRESSED: AtomicBool = AtomicBool::new(false);
static BTN_TOGGLE_PRESSED: AtomicBool = AtomicBool::new(false);

static MILLIS_CTR: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

// ── Helpers (unchanged) ──────────────────────────────────────

fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_CTR.borrow(cs).get())
}

fn take_encoder_delta() -> i8 {
    avr_device::interrupt::free(|cs| {
        let c = ENCODER_DELTA.borrow(cs);
        let v = c.get();
        c.set(0);
        v
    })
}

// ── Timer 1 — tone generation (unchanged) ────────────────────

const F_CPU: u32 = 16_000_000;
const PRESCALER: u32 = 8;

const fn ocr_for_dhz(freq_dhz: u32) -> u16 {
    let top = F_CPU * 10 / (2 * PRESCALER * freq_dhz) - 1;
    top as u16
}

fn set_tone_frequency(tc1: &TC1, freq_dhz: u16) {
    let ocr = ocr_for_dhz(freq_dhz as u32);
    tc1.tccr1a()
        .write(|w| w.com1a().match_toggle().wgm1().set(0b00));
    tc1.tccr1b()
        .write(|w| w.wgm1().set(0b01).cs1().prescale_8());
    tc1.ocr1a().write(|w| w.set(ocr));
    tc1.tcnt1().write(|w| w.set(0));
}

fn stop_tone(tc1: &TC1) {
    tc1.tccr1b().write(|w| w.cs1().no_clock());
    tc1.tccr1a().write(|w| w.com1a().disconnected());
}

// ── Millis timer (unchanged) ─────────────────────────────────

fn millis_init(tc0: atmega_hal::pac::TC0) {
    tc0.tccr0a().write(|w| w.wgm0().ctc());
    tc0.ocr0a().write(|w| w.set(249u8));
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

// ── Rotary encoder ISR (unchanged) ───────────────────────────

static mut LAST_ENC: u8 = 0;

fn encoder_init(exint: &atmega_hal::pac::EXINT) {
    exint.pcicr().write(|w| unsafe { w.bits(0b100) });
    exint.pcmsk2().write(|w| unsafe { w.bits(0b0000_1100) });
}

#[avr_device::interrupt(atmega328p)]
fn PCINT2() {
    let pind = unsafe { (*avr_device::atmega328p::PORTD::ptr()).pin().read().bits() };
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

// ── Display helpers (unchanged) ──────────────────────────────

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

// ── Clock type for I2C speed calculation ─────────────────────
type CoreClock = atmega_hal::clock::MHz8;
type I2c = atmega_hal::i2c::I2c<CoreClock>;

// ── Entry point ──────────────────────────────────────────────
#[avr_device::entry]
fn main() -> ! {
    let dp = atmega_hal::Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);

    // ── I2C bus for OLED (replaces entire SPI block) ─────────
    //  SDA = PC4,  SCL = PC5  (ATmega328P TWI hardware)
    let i2c = I2c::new(
        dp.TWI,
        pins.pc4.into_pull_up_input(), // SDA
        pins.pc5.into_pull_up_input(), // SCL
        400_000,                       // 400 kHz Fast-mode
    );

    // Build I2C display interface → ssd1306 driver in buffered graphics mode
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // ── Tone output pin (unchanged) ──────────────────────────
    let _tone_pin = pins.pb1.into_output();

    // ── Encoder + buttons (unchanged) ────────────────────────
    let _enc_a = pins.pd2.into_pull_up_input();
    let _enc_b = pins.pd3.into_pull_up_input();
    let btn_sel = pins.pd4.into_pull_up_input();
    let btn_tog = pins.pd7.into_pull_up_input();

    // ── Timers (unchanged) ───────────────────────────────────
    millis_init(dp.TC0);
    encoder_init(&dp.EXINT);

    let tc1 = dp.TC1;

    unsafe {
        avr_device::interrupt::enable();
    }

    // ── Application state (unchanged) ────────────────────────
    let mut tone_index: usize = 12;
    let mut output_on = false;
    let mut last_btn_sel = true;
    let mut last_btn_tog = true;
    let mut last_render: u32 = 0;
    let mut needs_redraw = true;

    stop_tone(&tc1);

    // ── Main loop ────────────────────────────────────────────
    loop {
        let now = millis();

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

        let sel_now = btn_sel.is_high();
        if last_btn_sel && !sel_now {
            BTN_SELECT_PRESSED.store(true, Ordering::SeqCst);
        }
        last_btn_sel = sel_now;

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

        if needs_redraw && now.wrapping_sub(last_render) >= 80 {
            needs_redraw = false;
            last_render = now;

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

            // ── Changed: ssd1306 clear + flush API ───────────
            display.clear_buffer();

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

            display.flush().unwrap();
        }

        delay_ms(2);
    }
}
