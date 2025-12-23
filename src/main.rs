#![no_std]
#![no_main]

use LSM6DSV32::lsm6dsv32::Lsm6dsv32;
use defmt::*;
use defmt_rtt as _;
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;

use embassy_executor::{Spawner, task};
use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, gpio::{Level, Output, Speed}, interrupt, mode::Async, pac::usb::vals::Stat, spi::{Config, Instance, Mode, Phase, Polarity, Spi}, time::Hertz, usart::{self, Config as UartConfig, Uart}
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use static_cell::StaticCell;

type SpiError = embassy_stm32::spi::Error;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Programm gestartet");

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(500_000);
    // CPOL = takt im Ruhezustand (0-> Idle = LOW; 1 -> Idle = HIGH)
    // CPHA = an welcher Flanke werden die Daten gelesen (0-> erste Flanke )
    spi_config.mode = Mode {
        polarity: Polarity::IdleLow,            // CPOL=0
        phase: Phase::CaptureOnFirstTransition, // CPHA=0
    };  // => SPI Mode 0

    static SPI: StaticCell<Spi<'static,Async>> = StaticCell::new();
    static CS: StaticCell<Output<'static>> = StaticCell::new();
    static INT1: StaticCell<ExtiInput<'static>> = StaticCell::new();
    static INT2: StaticCell<ExtiInput<'static>> = StaticCell::new();

    let spi = SPI.init(Spi::new(p.SPI1,
        p.PA5,
        p.PA7, 
        p.PA6, 
        p.DMA2_CH3, 
        p.DMA2_CH2, 
        spi_config));
    
    let cs = CS.init(Output::new(p.PB0, Level::High, Speed::High));

    let lsm = Lsm6dsv32::new(spi, cs, int1, int2);


}
