#![no_std]
#![no_main]

use core::{error, pin};

pub use crate::lsm6dsv32_config::*;

use cortex_m::delay;
use defmt::*;
use defmt_rtt as _;
use embassy_stm32::{
    can::config,
    exti::ExtiInput,
    gpio::{Level, Output, Speed},
    mode::Async,
    spi::{Config, Instance, Mode, Phase, Polarity, Spi},
    time::Hertz,
};
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;

type SpiError = embassy_stm32::spi::Error;

pub const EXPECTED_WHO_AM_I: u8 = 0x70;
pub const G32_SCALE_FACTOR: f32 = 32.0 / 32768.0;

#[macro_export]
macro_rules! encode_reg8 {
    (base: $base:expr, { $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        {
            let mut v: u8 = $base;
            $(
                let mask: u8 = ((1u16 << $width) - 1) as u8;
                let shifted_mask: u8 = mask << $shift;
                v = (v & !shifted_mask) | (($val as u8 & mask) << $shift);
            )*
            v
        }
    };

    ({ $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        $crate::encode_reg8!(base: 0, { $($val => $shift, $width),* })
    };
}



#[derive(defmt::Format)]
pub enum Error {
    Spi(SpiError),
    NoValue,
    WrongWhoAmI(u8),
}

pub struct lsm6dsv32_hw<'d> {
    spi: &'d mut Spi<'d, Async>,
    cs: &'d mut Output<'d>,
    int1: &'d mut ExtiInput<'d>,
    int2: &'d mut ExtiInput<'d>,
    accel_fs: f32,
    gyro_fs: f32,
    debug_mode: bool,
    bias_accel: [f32; 3],
    bias_gyro: [f32; 3],
    ts_offset: u32,
}

pub struct Lsm6dsv32<'d, F, I1, I2> {
    hw: lsm6dsv32_hw<'d>,
    config: ImuConfig<F, I1, I2>,
}

