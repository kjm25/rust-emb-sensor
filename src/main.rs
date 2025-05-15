#![no_main]
#![no_std]
#![allow(warnings)]


pub const RED: u8 = 0;
pub const GREEN: u8 = 1;
pub const BLUE: u8 = 2;

use microbit::{board, pac};
// use cortex_m_rt::entry;
use rtt_target::{rtt_init_print, rprintln};                                   
use panic_rtt_target as _;                                                    

use embedded_hal::{delay::DelayNs, digital::OutputPin, digital::InputPin};
use embedded_hal::digital::{ PinState };
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use bme280::spi::BME280;


use microbit::{
    board::Board,
    gpio::{MOSI, MISO, SCK},
    hal::gpio::{ Pin, Level, DriveConfig, OpenDrainConfig, Input, Output, PushPull, Floating, p0::P0_01, p0::P0_13, p0::P0_14, p0::P0_15},
    hal::timer::Timer,
    hal::Delay,
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
    let mut spi_timer = Timer::new(board.TIMER1);
    let mut bme_timer = Timer::new(board.TIMER2);

    let mut cs_pin = board.edge.e16.into_push_pull_output(Level::High);
    //let mi: P0_13<Input<Floating>>;
    //let peripherals = pac::Peripherals::take().unwrap();
    //let mut p0_parts = microbit::hal::gpio::p0::Parts::new(peripherals.P0);

    let mut sck =  board.pins.p0_17.into_push_pull_output(Level::Low);
    let mut miso  =  board.pins.p0_01.into_floating_input();
    let mut mosi =  board.pins.p0_13.into_push_pull_output(Level::Low);
    
    // let coretx_peripherals = cortex_m::Peripherals::take().unwrap();
    // let mut delay = Delay::new(coretx_peripherals.SYST);


    let p = Pins {
        sck:  core::prelude::v1::Some(sck.degrade()),
        mosi: core::prelude::v1::Some(mosi.degrade()),
        miso: core::prelude::v1::Some(miso.degrade()),
    };
    
    let s = Spi::new(
        board.SPI0, 
        p,
        Frequency::K250, 
        Mode{ polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition },  
    );

    let spi_dev = ExclusiveDevice::new(s, cs_pin, spi_timer);

    // Set up UARTE for microbit v2 using UartePort wrapper
    let mut serial = Uarte::new(
        board.UARTE0,
        board.uart.into(),
        Parity::EXCLUDED,
        Baudrate::BAUD115200,
    );

    // Clear terminal
    serial_write(&mut serial, b"\x1Bc");


    let mut bme280 = BME280::new(spi_dev.unwrap()).unwrap();
    bme280.init(&mut bme_timer).unwrap();
    
    
    loop {
        // measure temperature, pressure, and humidity
        let measurements= bme280.measure(&mut bme_timer).unwrap();

        rprintln!("Relative Humidity = {}%", measurements.humidity);
        rprintln!("Temperature = {} deg C", measurements.temperature);
        rprintln!("Pressure = {} pascals", measurements.pressure);

        // Off time
        timer.delay_ms(500);
    }
}
