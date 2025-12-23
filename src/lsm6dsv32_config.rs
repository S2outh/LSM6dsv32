#![no_std]
#![no_main]

pub struct ImuConfig {
    pub general: GenerelConfig,
    pub accel: AccelConfig,
    pub gyro: GyroConfig,
    pub fifo: FifoConfig,
    pub int1: Interrupt1Config,
    pub int2: Interrupt2Config,
}

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
    pub lpf1_enabled: bool,
    pub lpf1: GyroLpf1,
    pub full_scale: GyroFS,
}

pub struct AccelConfig {
    pub enabled: bool,
    pub mode: AccelOperatingMode,
    pub odr: AccelODR,
    pub lp_hp_f2: AccelFilterBW,
    pub full_scale: AccelFS,
    pub lp_hp: bool,
    pub lpf2_enabled: bool,
    pub hp_reference_mode: bool,
    pub user_offset_en: bool,
    pub user_offset_weight: bool,
    pub user_offset: [i8; 3],
    pub dual_channel: bool,
}

pub struct FifoConfig {
    pub mode: FIFOMode,
    pub watermark_threshold: u8,
    pub counter_threshold: u16,
    pub counter_trigger: TriggerCounter,
    pub gyro_fifo: GyroBatchDataRate,
    pub accel_fifo: AccelBatchDataRate,
    pub temp_fifo: TempatureBatchRate,
    pub ts_fifo: TimeStampBatch,
}

pub struct Interrupt1Config {
    pub counter_bdr_int: bool,
    pub fifo_full_int: bool,
    pub fifo_overrun_int: bool,
    pub fifo_threshold_int: bool,
    pub data_ready_gyro: bool,
    pub data_ready_accel: bool,
}

pub struct Interrupt2Config {
    pub counter_bdr_int: bool,
    pub fifo_full_int: bool,
    pub fifo_overrun_int: bool,
    pub fifo_threshold_int: bool,
    pub data_ready_gyro: bool,
    pub data_ready_accel: bool,
    pub temp_ready: bool,
}
impl AccelConfig {
    pub fn calc_scaling_factor(&self) -> f32 {
        match self.full_scale {
            AccelFS::G4 => 4.0 / 32768.0,
            AccelFS::G8 => 8.0 / 32768.0,
            AccelFS::G16 => 16.0 / 32768.0,
            AccelFS::G32 => 32.0 / 32768.0,
        }
    }
}

impl GyroConfig {
    pub fn calc_scaling_factor(&self) -> f32 {
        match self.full_scale {
            GyroFS::DPS125 => 125.0 / 32768.0,
            GyroFS::DPS250 => 250.0 / 32768.0,
            GyroFS::DPS500 => 500.0 / 32768.0,
            GyroFS::DPS1000 => 1000.0 / 32768.0,
            GyroFS::DPS2000 => 2000.0 / 32768.0,
            GyroFS::DPS4000 => 4000.0 / 32768.0,
        }
    }
}
// Gyro Output-Data-Rate
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GyroODR {
    PowerDown = 0b0000,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GyroLpf1 {
    ExtraWide = 0b000,
    Wide = 0b001,
    MediumWide = 0b010,
    Medium = 0b011,
    MediumNarrow = 0b100,
    Narrow = 0b101,
    VeryNarrow = 0b110,
    UltraNarrow = 0b111,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GyroFS {
    DPS125 = 0b0000,
    DPS250 = 0b0001,
    DPS500 = 0b0010,
    DPS1000 = 0b0011,
    DPS2000 = 0b0100,
    DPS4000 = 0b1100,
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GyroOperatingMode {
    HighPerformance,
    HighAccuracy,
    ODRTriggered,
    LowPowerMode,
}

// Rate at which gyro data fills the FIFO
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GyroBatchDataRate {
    NotBatched = 0b0000,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}
// Accel Output-Data-Rate
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum AccelODR {
    PowerDown = 0b0000,
    Hz1_875 = 0b0001,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum AccelFilterBW {
    OdrDiv4 = 0b000,
    OdrDiv10 = 0b001,
    OdrDiv20 = 0b010,
    OdrDiv45 = 0b011,
    OdrDiv100 = 0b100,
    OdrDiv200 = 0b101,
    OdrDiv400 = 0b110,
    OdrDiv800 = 0b111,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum AccelFS {
    G4 = 0b00,
    G8 = 0b01,
    G16 = 0b10,
    G32 = 0b11,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum AccelOperatingMode {
    HighPerformance = 0b000,
    HighAccuracy = 0b001,
    ODRTriggered = 0b011,
    // takes mean value of x samples
    LowPowerMode2Mean = 0b100,
    LowPowerMode4Mean = 0b101,
    LowPowerMode8Mean = 0b110,
    Normal = 0b111,
}
// Rate at which accel data fills the FIFO
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum AccelBatchDataRate {
    NotBatched = 0b0000,
    Hz1_875 = 0b0001,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}

// Configures decimation for timestamp batching in FIFO to control the write rate relative to the maximum sensor data rate
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum TimeStampBatch {
    NotBatched = 0b00,
    Every1 = 0b01,
    Every8 = 0b10,
    Every32 = 0b11,
}

// Rate at which temp data fills the FIFO
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum TempatureBatchRate {
    NotBatched = 0b00,
    Hz1_875 = 0b01,
    Hz15 = 0b10,
    Hz60 = 0b11,
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum FIFOMode {
    Bypass = 0b000,                 // FIFO disabled
    FifoMode = 0b001,               // Stops collecting data once the buffer is full
    ContinuousToFullMode = 0b010,   // Continuous mode until trigger, then fills buffer to capacity
    ContinuousToFifoMode = 0b011,   // Continuous mode until trigger, then switches to FifoMode
    BypassToContinuousMode = 0b100, // Fifo disabled until trigger, then starts continuous data collection
    ContinuousMode = 0b110,         // new samples overwrite the oldest once the fifo is full
    BypassToFifoMode = 0b111,       // Fifo disabled until trigger, then fills once and stops
}
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum TriggerCounter {
    Accel = 0b00,
    Gyro = 0b01,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            general: GenerelConfig {
                sda_pull_up: false,
                sdo_pull_up: false,
                anti_spike_filter: false,
                interrupt_lvl: false,
                interrupt_pin_mode: false,
            },
            accel: AccelConfig {
                enabled: true,
                mode: AccelOperatingMode::HighPerformance,
                odr: AccelODR::Hz960,
                lp_hp_f2: AccelFilterBW::OdrDiv4,
                full_scale: AccelFS::G4,
                lp_hp: false,
                lpf2_enabled: false,
                hp_reference_mode: false,
                user_offset_en: false,
                user_offset_weight: false,
                user_offset: [0; 3],
                dual_channel: false,
            },
            gyro: GyroConfig {
                enabled: true,
                mode: GyroOperatingMode::HighPerformance,
                odr: GyroODR::Hz960,
                lpf1_enabled: false,
                lpf1: GyroLpf1::ExtraWide,
                full_scale: GyroFS::DPS125,
            },
            fifo: FifoConfig {
                mode: FIFOMode::Bypass,
                watermark_threshold: 0,
                counter_threshold: 0,
                counter_trigger: TriggerCounter::Accel,
                gyro_fifo: GyroBatchDataRate::NotBatched,
                accel_fifo: AccelBatchDataRate::NotBatched,
                temp_fifo: TempatureBatchRate::NotBatched,
                ts_fifo: TimeStampBatch::NotBatched,
            },
            int1: Interrupt1Config {
                counter_bdr_int: false,
                fifo_full_int: false,
                fifo_overrun_int: false,
                fifo_threshold_int: false,
                data_ready_gyro: false,
                data_ready_accel: false,
            },
            int2: Interrupt2Config {
                counter_bdr_int: false,
                fifo_full_int: false,
                fifo_overrun_int: false,
                fifo_threshold_int: false,
                data_ready_gyro: false,
                data_ready_accel: false,
                temp_ready: false,
            },
        }
    }
}
