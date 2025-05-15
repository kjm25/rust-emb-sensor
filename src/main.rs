#![no_main]
#![no_std]
#![allow(warnings)]


pub const RED: u8 = 0;
pub const GREEN: u8 = 1;
pub const BLUE: u8 = 2;

// use cortex_m_rt::entry;
use rtt_target::{rtt_init_print, rprintln};                                   
use panic_rtt_target as _;                                                    

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use embedded_hal::digital::{ PinState, InputPin };

use bme280::spi::BME280;


use microbit::{
    board::Board,
    gpio::{MOSI, MISO, SCK},
    hal::gpio::{ Pin, Level, DriveConfig, OpenDrainConfig, Input, Output, PushPull, Floating, p0::P0_01, p0::P0_13, p0::P0_17},
    hal::timer::Timer,
    hal::uarte::{Uarte, Baudrate, Parity},
    hal::spi::{Spi, Pins, Frequency},
    hal::spim::{MODE_0, Polarity, Phase, Mode},
};


fn serial_write<T>(serial: &mut Uarte<T>, buffer: &[u8]) -> () where T: microbit::hal::uarte::Instance {

    // for each byte in the buffer, write it out to the serial port
    for b in buffer {
        match serial.write(&[*b]) {
            Ok(_r) => (),
            Err(e) => rprintln!("Serial Error: {:?}", e),
        }
    }    
}

#[cortex_m_rt::entry]
fn start_here() -> ! {

    rtt_init_print!();
    let mut board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);

    let mut cs_pin = board.edge.e16.into_push_pull_output(Level::High);
    // let mi: P0_13<Input<Floating>>;
    // let s: P0_17<Output<PushPull>>;
    // let mut miso   = mi.into_floating_input();
    // let mut sck    = s.into_push_pull_output(Level::Low);

    let p = Pins {
        sck:  core::prelude::v1::Some(Pin::into_push_pull_output(Level::Low)),
        mosi: core::prelude::v1::Some(Pin::into_push_pull_output(Level::Low)),
        miso: core::prelude::v1::Some(Pin::into_floating_input()),
    };
    
    let s = Spi::new(
        board.SPI0, 
        p,
        Frequency::K250, 
        Mode{ polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition },  
    );

    // Set up UARTE for microbit v2 using UartePort wrapper
    let mut serial = Uarte::new(
        board.UARTE0,
        board.uart.into(),
        Parity::EXCLUDED,
        Baudrate::BAUD115200,
    );

    // Clear terminal
    serial_write(&mut serial, b"\x1Bc");
    
    // Setup the low side of the matrix LED
    board.display_pins.col5.set_state(PinState::Low).unwrap();
    
    // Setup the RGB LED pins as open drain with inital setting LOW
    // Turns off the RGB LED
    let mut red_left = board.edge.e00.into_open_drain_output(OpenDrainConfig::HighDrive0Disconnect1,Level::Low);
    let mut green_left = board.edge.e01.into_open_drain_output(OpenDrainConfig::HighDrive0Disconnect1,Level::Low);
    let mut blue_left = board.edge.e02.into_open_drain_output(OpenDrainConfig::HighDrive0Disconnect1,Level::Low);

    // Initial color state
    let mut color_state = RED;

    loop {

        // Toggle the Matrix LED on
        board.display_pins.row3.set_state(PinState::High).unwrap();
        
        // depending on color state, turn on the RGB LED
        if color_state == RED   {red_left.set_high().unwrap(); serial_write(&mut serial, b"Red\n\r"); }
        if color_state == GREEN {green_left.set_high().unwrap(); serial_write(&mut serial, b"Green\n\r"); }
        if color_state == BLUE  {blue_left.set_high().unwrap(); serial_write(&mut serial, b"Blue\n\r"); }

        // On time
        timer.delay_ms(500);

        // Toggle the Matrix LED off
        board.display_pins.row3.set_state(PinState::Low).unwrap();

        // depending on color state, turn off the RGB LED
        if color_state == RED   {red_left.set_low().unwrap();}
        if color_state == GREEN {green_left.set_low().unwrap();}
        if color_state == BLUE  {blue_left.set_low().unwrap();}

        // Change color state to switch between RGB LEDs
        if color_state > 1 {
            color_state = RED;
            serial_write(&mut serial, b"\x1Bc");
        } else {
            color_state += 1;
        }

        // Off time
        timer.delay_ms(500);
    }
}
