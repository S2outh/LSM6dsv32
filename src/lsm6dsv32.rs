#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;

use embassy_stm32::{
    gpio::{Level, Output, Speed},
    mode::Async,
    spi::{Config, Instance, Mode, Phase, Polarity, Spi},
    time::Hertz,
};

use bitflags::bitflags;

type SpiError = embassy_stm32::spi::Error;

const PIN_CTRL: u8 = 0x02;
const IF_CTRL: u8 = 0x03;
const FIFO_CTRL1: u8 = 0x07;
const FIFO_CTRL2: u8 = 0x08;
const FIFO_CTRL3: u8 = 0x09;
const FIFO_CTRL4: u8 = 0x0A;
const COUNTER_BDR_REG1: u8 = 0x0B;
const COUNTER_BDR_REG2: u8 = 0x0C;
const WHO_AM_I: u8 = 0x0F;
const INT1_CTRL: u8 = 0x0D;
const INT2_CTRL: u8 = 0x0E;
const CTRL1: u8 = 0x10;
const CTRL2: u8 = 0x11;
const CTRL3: u8 = 0x12;
const CTRL4: u8 = 0x13;
const CTRL5: u8 = 0x14;
const CTRL6: u8 = 0x15;
const CTRL7: u8 = 0x16;
const CTRL8: u8 = 0x17;
const CTRL9: u8 = 0x18;
const CTRL10: u8 = 0x19;
const OUT_TEMP_L: u8 = 0x20;
const OUTX_L_G: u8 = 0x22;
const UI_OUTX_L_G_OIS_EIS: u8 = 0x2E;
const UI_OUTX_L_A_OIS_DUAL_C: u8 = 0x34;
const TIMESTAMP0: u8 = 0x40;
const WAKE_UP_SRC: u8 = 0x45;
const WAKE_UP_THS: u8 = 0x5B;
const WAKE_UP_DUR: u8 = 0x5C;
const MD1_CFG: u8 = 0x5E;
const MD2_CFG: u8 = 0x5F;
const X_Y_Z_IFS_USR: u8 = 0x73;
const FIFO_DATA_OUT_TAG: u8 = 0x78;
const FIFO_DATA_OUT_X_L: u8 = 0x79;

#[derive(defmt::Format)]
pub enum Error {
    Spi(SpiError),
}

pub struct Lsm6dsv32<'d> {
    spi: &'d mut Spi<'d, Async>,
    cs: &'d mut Output<'d>,
    accel_fs: f32,
    gyro_fs: f32,
    config: Option<ImuConfig>,
    bias_accel: [f32; 3],
    bias_gyro: [f32; 3],
}

pub struct ImuConfig {}


pub struct GenerelConfig {
    pub sda_pull_up: bool,
    pub sdo_pull_up: bool,
    pub anti_spike_filter: bool,
    pub interrupt_lvl: bool,
    pub interrupt_pin_mode: bool,
}


pub struct GyroConfig {
    pub enabled: bool,
    pub mode: GyroOperatingMode,
    pub odr: GyroODR,
    pub low_pass_filter: GyroLpf1,
    pub full_scale: GyroFS
}

pub struct AccelConfig {
    pub enabled: bool,
    pub mode: AccelOperatingMode,
    pub odr: AccelODR,
    pub low_pass_filter: AccelFilterBW,
    pub full_scale: AccelFS,
    pub hp_lp: bool,
    pub lpf2_enabled: bool,
    pub hp_reference_mode: bool,
    pub user_offset_en: bool,
    pub user_offset_weight: bool,
}

pub struct FifoConfig {
    pub mode: FIFOMode,
    pub gyro_fifo: GyroBatchDataRate,
    pub accel_fifo: AccelBatchDataRate,
    pub temp_fifo: TempatureBatchRate,
    pub ts_fifo: TimeStampBatch,
}

pub struct Interrupt1Config {

}

#[macro_export]
macro_rules! reg8 {
    (
        $(#[$meta:meta])*
        struct $name:ident ($addr:expr) {
            $($field:ident : $fty:ty => $shift:literal , $width:literal = $value:expr),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        pub struct $name {
            $(pub $field: $fty),*
        }

        impl Default for $name {
            fn default() -> Self {
                Self {
                    $($field: $value),*
                }
            }
        }

        impl $name {
            pub const ADDR: u8 = $addr;


            #[inline]
            pub fn encode(&self) -> u8 {
                let mut v: u8 = 0;
                $(
                    let basemask: u8 = ((1u16 << $width)-1) as u8;
                    let shifted_mask: u8 = basemask << $shift;
                    let value: u8 = (self.$field as u8) & basemask;
                    v = (v & !shifted_mask) | ((value << $shift) & shifted_mask);
                )*
                v
            }
        }


    };

    (
        $(#[$meta:meta])*
        $name:ident ($addr:expr) {
            $($field:ident : $fty:ty => $shift:literal , $width:literal = $value:expr),* $(,)?
        }

    ) => {
        {
            reg8!($(#[$meta])* struct $name ($addr) { $($field : $fty => $shift , $width = $value),* });
            $name::default()
        }
    };
}

bitflags! {
    pub struct InnterfaceConfig: u8 {
        const SDA_PULLUP_ENABLE = 1 << 7;
        const ANTISPIKE_FILTER = 1 << 5;
        const INTERRUPT_LVL = 1 << 4;
        const INT_PIN_MODE = 1 << 3;
    }

    pub struct Interrupt1Ctrl: u8 {
        const COUNTER_BDR_INT = 1 << 6;
        const FIFO_FULL_INT = 1 << 5;
        const FIFO_OVERRUN_INT = 1 << 4;
        const FIFO_THRESHOLD_INT = 1 << 3;
        const DATA_READY_GYRO = 1 << 1;
        const DATA_READY_ACCEL = 1 << 0;
    }

    pub struct Interrupt1Ctr2: u8 {
        const COUNTER_BDR_INT = 1 << 6;
        const FIFO_FULL_INT = 1 << 5;
        const FIFO_OVERRUN_INT = 1 << 4;
        const FIFO_THRESHOLD_INT = 1 << 3;
        const DATA_READY_GYRO = 1 << 1;
        const DATA_READY_ACCEL = 1 << 0;
    }

    pub struct Ctrl9Reg: u8 {
        const HP_REFERENCE_MODE = 1 << 6;
        const ACCEL_HP_LP = 1 << 4;
        const LPF2_EN = 1 << 3;
        const USER_OFFSET_WEIGHT = 1 << 1;
        const ACCEL_USER_OFFSET_EN = 1;
    }
}

// Gyro Output-Data-Rate
#[derive(Copy, Clone, Debug)]
pub enum GyroODR {
    PowerDown       = 0b0000,
    Hz7_5           = 0b0010,
    Hz15            = 0b0011,
    Hz30            = 0b0100,
    Hz60            = 0b0101,
    Hz120           = 0b0110,
    Hz240           = 0b0111,
    Hz480           = 0b1000,
    Hz960           = 0b1001,
    KHz1_92         = 0b1010,
    KHz3_84         = 0b1011,
    KHz7_68         = 0b1100,
}
#[derive(Copy, Clone, Debug)]
pub enum GyroLpf1 {
    ExtraWide           = 0b000,
    Wide                = 0b001,
    MediumWide          = 0b010,
    Medium              = 0b011,
    MediumNarrow        = 0b100,
    Narrow              = 0b101,
    VeryNarrow          = 0b110,
    UltraNarrow         = 0b111,
}
#[derive(Copy, Clone, Debug)]
pub enum GyroFS {
    DPS125      = 0b0000,
    DPS250      = 0b0001,
    DPS500      = 0b0010,
    DPS1000     = 0b0011,
    DPS2000     = 0b0100,
    DPS4000     = 0b1100,
}

#[derive(Copy, Clone, Debug)]
pub enum GyroOperatingMode {
    HighPerformance,
    HighAccuracy,
    ODRTriggered,
    LowPowerMode,
}

// Rate at which gyro data fills the FIFO
#[derive(Copy, Clone, Debug)]
pub enum GyroBatchDataRate {
    NotBatched  = 0b0000,
    Hz7_5       = 0b0010,
    Hz15        = 0b0011,
    Hz30        = 0b0100,
    Hz60        = 0b0101,
    Hz120       = 0b0110,
    Hz240       = 0b0111,
    Hz480       = 0b1000,
    Hz960       = 0b1001,
    KHz1_92     = 0b1010,
    KHz3_84     = 0b1011,
    KHz7_68     = 0b1100,
}
// Accel Output-Data-Rate
#[derive(Copy, Clone, Debug)]
pub enum AccelODR {
    PowerDown       = 0b0000,
    Hz1_875         = 0b0001,
    Hz7_5           = 0b0010,
    Hz15            = 0b0011,
    Hz30            = 0b0100,
    Hz60            = 0b0101,
    Hz120           = 0b0110,
    Hz240           = 0b0111,
    Hz480           = 0b1000,
    Hz960           = 0b1001,
    KHz1_92         = 0b1010,
    KHz3_84         = 0b1011,
    KHz7_68         = 0b1100,
}
#[derive(Copy, Clone, Debug)]
pub enum AccelFilterBW {
    OdrDiv4     = 0b000,
    OdrDiv10    = 0b001,
    OdrDiv20    = 0b010,
    OdrDiv45    = 0b011,
    OdrDiv100   = 0b100,
    OdrDiv200   = 0b101,
    OdrDiv400   = 0b110,
    OdrDiv800   = 0b111,
}
#[derive(Copy, Clone, Debug)]
pub enum AccelFS {
    G4      = 0b00,
    G8      = 0b01,
    G16     = 0b10,
    G32     = 0b11,
}
#[derive(Copy, Clone, Debug)]
pub enum AccelOperatingMode {
    HighPerformance     = 0b000,
    HighAccuracy        = 0b001,
    ODRTriggered        = 0b011,
    // takes mean value of x samples
    LowPowerMode2Mean   = 0b100,
    LowPowerMode4Mean   = 0b101,
    LowPowerMode8Mean   = 0b110,
    Normal              = 0b111,
}
// Rate at which accel data fills the FIFO
#[derive(Copy, Clone, Debug)]
pub enum AccelBatchDataRate {
    NotBatched  = 0b0000,
    Hz1_875     = 0b0001,
    Hz7_5       = 0b0010,
    Hz15        = 0b0011,
    Hz30        = 0b0100,
    Hz60        = 0b0101,
    Hz120       = 0b0110,
    Hz240       = 0b0111,
    Hz480       = 0b1000,
    Hz960       = 0b1001,
    KHz1_92     = 0b1010,
    KHz3_84     = 0b1011,
    KHz7_68     = 0b1100,
}

// Configures decimation for timestamp batching in FIFO to control the write rate relative to the maximum sensor data rate
#[derive(Copy, Clone, Debug)]
pub enum TimeStampBatch {
    NotBatched  = 0b00,
    Every1      = 0b01,
    Every8      = 0b10,
    Every32     = 0b11,
}

// Rate at which temp data fills the FIFO
#[derive(Copy, Clone, Debug)]
pub enum TempatureBatchRate {
    NotBatched  = 0b00,
    Hz1_875     = 0b01,
    Hz15        = 0b10,
    Hz60        = 0b11,
}
#[derive(Copy, Clone, Debug)]
pub enum FIFOMode {
    Bypass                  = 0b000,    // FIFO disabled
    FifoMode                = 0b001,    // Stops collecting data once the buffer is full
    ContinuousToFullMode    = 0b010,    // Continuous mode until trigger, then fills buffer to capacity
    ContinuousToFifoMode    = 0b011,    // Continuous mode until trigger, then switches to FifoMode
    BypassToContinuousMode  = 0b100,    // Fifo disabled until trigger, then starts continuous data collection
    ContinuousMode          = 0b110,    // new samples overwrite the oldest once the fifo is full
    BypassToFifoMode        = 0b111,    // Fifo disabled until trigger, then fills once and stops
}
#[derive(Copy, Clone, Debug)]
pub enum TriggerCounter {
    Accel   = 0b00,
    Gyro    = 0b01,
}