#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;

use embassy_executor::{Spawner, task};
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    interrupt,
    mode::Async,
    spi::{Config, Instance, Mode, Phase, Polarity, Spi},
    time::Hertz,
    usart,
    usart::{Config as UartConfig, Uart},
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use static_cell::StaticCell;

type SpiError = embassy_stm32::spi::Error;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Programm gestartet");
}
