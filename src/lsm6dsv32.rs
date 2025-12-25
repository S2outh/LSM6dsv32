#![no_std]
#![no_main]

use core::pin;

pub use crate::lsm6dsv32_config::*;

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

pub struct ImuSample {
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
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

impl<'d> Lsm6dsv32<'d, FifoDisabled, Int1Disabled, Int2Disabled> {
    pub async fn new(
        spi: &'d mut Spi<'d, Async>,
        cs: &'d mut Output<'d>,
        int1: &'d mut ExtiInput<'d>,
        int2: &'d mut ExtiInput<'d>,
        debug: bool,
    ) -> Self {
        let imu_config = ImuConfig::default();

        let hw = lsm6dsv32_hw {
            spi,
            cs,
            int1,
            int2,
            accel_fs: 0.0,
            gyro_fs: 0.0,
            debug_mode: debug,
            bias_accel: [0.0; 3],
            bias_gyro: [0.0; 3],
            ts_offset: 0,
        };
        let mut instance = Self {
            hw,
            config: imu_config.clone(),
        };
        instance.reset_timer().await;
        instance.set_config(imu_config.build()).await;
        instance
    }
}

impl<'d, L, I1, I2> Lsm6dsv32<'d, L, I1, I2> {
    pub fn enable_debug(&mut self, enable: bool) {
        self.hw.debug_mode = enable;
    }

    async fn set_config(&mut self, imu_config: ImuConfigRaw) {
        if let Err(e) = self.write_config_registers(&imu_config).await {
            error!("Error writing config registers: {:?}", e);
        }

        if let Err(e) = self.check_who_i_am().await {
            error!("Error checking WHO_AM_I register: {:?}", e);
        }

        let cfg = &self.config;
        self.hw.accel_fs = cfg.accel.calc_scaling_factor();
        self.hw.gyro_fs = cfg.gyro.calc_scaling_factor();
        info!("IMU-configuration finished");
    }

    async fn check_who_i_am(&mut self) -> Result<(), Error> {
        let who_am_i_data = self.read_register(Register::WHO_AM_I as u8).await?;
        if who_am_i_data != EXPECTED_WHO_AM_I {
            return Err(Error::WrongWhoAmI(who_am_i_data));
        }
        Ok(())
    }

    async fn update_config(&mut self, registers: &[Register]) -> Result<(), Error> {
        let imu_config = self.config.build();
        self.write_config_registers(&imu_config).await?;
        Ok(())
    }

    async fn write_config_registers(&mut self, imu_config: &ImuConfigRaw) -> Result<(), Error> {
        let debug = self.hw.debug_mode;

        macro_rules! log_reg {
            ($name:expr, $val:expr) => {
                if debug {
                    debug!("{}: {:08b}", $name, $val);
                }
            };
            ($name:expr, $old:expr, $new:expr) => {
                if debug {
                    debug!("{} old: {:08b}, new: {:08b}", $name, $old, $new);
                }
            };
        }


        let pin_ctrl_old = self.read_register(Register::PIN_CTRL as u8).await?;
        let pin_ctrl = encode_reg8!(base: pin_ctrl_old, {
            1 => 0, 2,                                     
            1 => 5, 1,                                      
            imu_config.general.sdo_pull_up as u8 => 6, 1,   
            0 => 7, 1                                       
        });
        log_reg!("PIN_CTRL", pin_ctrl_old, pin_ctrl);
        self.write_register(Register::PIN_CTRL as u8, pin_ctrl)
            .await?;


        let if_ctrl_old = self.read_register(Register::IF_CTRL as u8).await?;
        let if_ctrl = encode_reg8!(base: if_ctrl_old, {
            imu_config.general.sda_pull_up as u8 => 7, 1,
            imu_config.general.anti_spike_filter as u8 => 5, 1,
            imu_config.general.interrupt_lvl as u8 => 4, 1,
            imu_config.general.interrupt_pin_mode as u8 => 3, 1,
            1 => 0, 1                                       // set
        });
        log_reg!("IF_CTRL", if_ctrl_old, if_ctrl);
        self.write_register(Register::IF_CTRL as u8, if_ctrl)
            .await?;


        let fifo_ctrl1 = encode_reg8!({
            imu_config.fifo.watermark_threshold => 0, 8
        });
        log_reg!("FIFO_CTRL1", fifo_ctrl1);
        self.write_register(Register::FIFO_CTRL1 as u8, fifo_ctrl1)
            .await?;


        let fifo_ctrl3 = encode_reg8!({
            imu_config.fifo.gyro_fifo as u8 => 4, 4,
            imu_config.fifo.accel_fifo as u8 => 0, 4
        });
        log_reg!("FIFO_CTRL3", fifo_ctrl3);
        self.write_register(Register::FIFO_CTRL3 as u8, fifo_ctrl3)
            .await?;


        let fifo_ctrl4 = encode_reg8!({
            imu_config.fifo.ts_fifo as u8 => 6, 2,
            imu_config.fifo.temp_fifo as u8 => 4, 2,
            imu_config.fifo.mode as u8 => 0, 3
        });
        log_reg!("FIFO_CTRL4", fifo_ctrl4);
        self.write_register(Register::FIFO_CTRL4 as u8, fifo_ctrl4)
            .await?;


        let counter_th_89 = ((imu_config.fifo.counter_threshold >> 8) & 0b11) as u8;
        let counter_th_07 = (imu_config.fifo.counter_threshold & 0xFF) as u8;

        let cbdr1 = encode_reg8!({
            imu_config.fifo.counter_trigger as u8 => 5, 2,
            counter_th_89 => 0, 2
        });
        self.write_register(Register::COUNTER_BDR_REG1 as u8, cbdr1)
            .await?;

        let cbdr2 = encode_reg8!({ counter_th_07 => 0, 8 });
        self.write_register(Register::COUNTER_BDR_REG2 as u8, cbdr2)
            .await?;


        let int1 = encode_reg8!({
            imu_config.int1.counter_bdr_int as u8 => 6, 1,
            imu_config.int1.fifo_full_int as u8 => 5, 1,
            imu_config.int1.fifo_overrun_int as u8 => 4, 1,
            imu_config.int1.fifo_threshold_int as u8 => 3, 1,
            imu_config.int1.data_ready_gyro as u8 => 1, 1,
            imu_config.int1.data_ready_accel as u8 => 0, 1
        });
        self.write_register(Register::INT1_CTRL as u8, int1).await?;


        let int2 = encode_reg8!({
            imu_config.int2.counter_bdr_int as u8 => 6, 1,
            imu_config.int2.fifo_full_int as u8 => 5, 1,
            imu_config.int2.fifo_overrun_int as u8 => 4, 1,
            imu_config.int2.fifo_threshold_int as u8 => 3, 1,
            imu_config.int2.data_ready_gyro as u8 => 1, 1,
            imu_config.int2.data_ready_accel as u8 => 0, 1
        });
        self.write_register(Register::INT2_CTRL as u8, int2).await?;


        let c1 = encode_reg8!({
            imu_config.accel.mode as u8 => 4, 3,
            imu_config.accel.odr as u8 => 0, 4
        });
        self.write_register(Register::CTRL1 as u8, c1).await?;


        let c2 = encode_reg8!({
            imu_config.gyro.mode as u8 => 4, 3,
            imu_config.gyro.odr as u8 => 0, 4
        });
        self.write_register(Register::CTRL2 as u8, c2).await?;


        let c4_old = self.read_register(Register::CTRL4 as u8).await?;
        let c4 = encode_reg8!(base: c4_old, {
            imu_config.int2.temp_ready as u8 => 2, 1
        });
        self.write_register(Register::CTRL4 as u8, c4).await?;


        let c6 = encode_reg8!({
            imu_config.gyro.lpf1 as u8 => 4, 3,
            imu_config.gyro.full_scale as u8 => 0, 4
        });
        self.write_register(Register::CTRL6 as u8, c6).await?;


        let c7 = encode_reg8!({ imu_config.gyro.lpf1_enabled as u8 => 0, 1 });
        self.write_register(Register::CTRL7 as u8, c7).await?;


        let c8 = encode_reg8!({
            imu_config.accel.lp_hp_f2 as u8 => 5, 3,
            imu_config.accel.dual_channel as u8 => 3, 1,
            1 => 2, 1,                                   
            imu_config.accel.full_scale as u8 => 0, 2
        });
        self.write_register(Register::CTRL8 as u8, c8).await?;


        let c9_old = self.read_register(Register::CTRL9 as u8).await?;
        let c9 = encode_reg8!(base: c9_old, {
            imu_config.accel.hp_reference_mode as u8 => 6, 1,
            imu_config.accel.lp_hp as u8 => 4, 1,
            imu_config.accel.lpf2_enabled as u8 => 3, 1,
            imu_config.accel.user_offset_weight as u8 => 1, 1,
            imu_config.accel.user_offset_en as u8 => 0, 1
        });
        self.write_register(Register::CTRL9 as u8, c9).await?;


        self.write_register(0x73, imu_config.accel.user_offset[0] as u8)
            .await?;
        self.write_register(0x74, imu_config.accel.user_offset[1] as u8)
            .await?;
        self.write_register(0x75, imu_config.accel.user_offset[2] as u8)
            .await?;


        let fe = encode_reg8!({
            imu_config.general.timestamp_enabled as u8 => 6, 1
        });
        self.write_register(Register::FUNCTIONS_ENABLE as u8, fe)
            .await?;

        Ok(())
    }

    async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg & !READ_BIT; //Bit 7 = 0 -> Write-Mode -> 0x80: 1000 0000 -> !0x80: 0111 1111

        let mut buffer = [cmd, val];
        self.hw.cs.set_low();
        self.hw
            .spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.hw.cs.set_high();

        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg | READ_BIT;

        let mut buffer = [cmd, 0x00];

        self.hw.cs.set_low();
        self.hw
            .spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.hw.cs.set_high();

        if buffer.len() < 1 {
            return Err(Error::NoValue);
        } else {
            Ok(buffer[1])
        }
    }

    async fn read_multi_registers(&mut self, start_reg: u8, data: &mut [u8]) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = start_reg | READ_BIT;

        let mut buffer = [0u8; 9]; //max 8 Bytes data
        let n = 1 + data.len();
        buffer[0] = cmd;

        self.hw.cs.set_low();
        self.hw
            .spi
            .transfer_in_place(&mut buffer[..n])
            .await
            .map_err(Error::Spi)?;
        self.hw.cs.set_high();

        data.copy_from_slice(&buffer[1..n]);
        Ok(())
    }

    async fn read_accel_raw(&mut self) -> Result<[i16; 3], Error> {
        let mut data = [0u8; 6];
        self.read_multi_registers(Register::OUTX_L_A as u8, &mut data)
            .await?;
        let ax = i16::from_le_bytes([data[0], data[1]]);
        let ay = i16::from_le_bytes([data[2], data[3]]);
        let az = i16::from_le_bytes([data[4], data[5]]);
        Ok([ax, ay, az])
    }

    async fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error> {
        let mut data = [0u8; 6];
        self.read_multi_registers(Register::OUTX_L_G as u8, &mut data)
            .await?;
        let gx = i16::from_le_bytes([data[0], data[1]]);
        let gy = i16::from_le_bytes([data[2], data[3]]);
        let gz = i16::from_le_bytes([data[4], data[5]]);
        Ok([gx, gy, gz])
    }

    async fn read_temp_raw(&mut self) -> Result<i16, Error> {
        let mut data = [0u8; 2];
        self.read_multi_registers(Register::OUT_TEMP_L as u8, &mut data)
            .await?;
        let temp = i16::from_le_bytes([data[0], data[1]]);
        Ok(temp)
    }

    async fn read_timestamp_raw(&mut self) -> Result<u32, Error> {
        let mut data = [0u8; 4];
        self.read_multi_registers(Register::TIMESTAMP0 as u8, &mut data)
            .await?;
        Ok(u32::from_le_bytes(data))
    }

    pub async fn read_temp(&mut self) -> Result<f32, Error> {
        let raw = self.read_temp_raw().await?;
        Ok((raw as f32 / 256.0) + 25.0)
    }

    pub async fn read_timestamp(&mut self) -> Result<f32, Error> {
        let raw = self.read_timestamp_raw().await?;
        Ok((raw - self.hw.ts_offset) as f32 * 21.75e-3)
    }

    pub async fn read_accel(&mut self) -> Result<[f32; 3], Error> {
        let raw = self.read_accel_raw().await?;
        let scale = self.hw.accel_fs;
        debug!("Accel raw: {:?}, scale: {}", raw, scale);
        Ok([
            (raw[0] as f32 - self.hw.bias_accel[0]) * scale,
            (raw[1] as f32 - self.hw.bias_accel[1]) * scale,
            (raw[2] as f32 - self.hw.bias_accel[2]) * scale,
        ])
    }

    pub async fn read_gyro(&mut self) -> Result<[f32; 3], Error> {
        let raw = self.read_gyro_raw().await?;
        let scale = self.hw.gyro_fs;
        Ok([
            (raw[0] as f32 - self.hw.bias_gyro[0]) * scale,
            (raw[1] as f32 - self.hw.bias_gyro[1]) * scale,
            (raw[2] as f32 - self.hw.bias_gyro[2]) * scale,
        ])
    }

    pub async fn read_all(&mut self) -> Result<ImuSample, Error> {
        let gyro = self.read_gyro().await?;
        let accel = self.read_accel().await?;

        Ok(ImuSample { accel, gyro })
    }

    pub async fn reset_timer(&mut self){
        info!("Resetting timestamp timer");
        self.hw.ts_offset = self.read_timestamp_raw().await.unwrap_or(0);
    }
}

impl<'d, I1, I2> Lsm6dsv32<'d, FifoDisabled, I1, I2> {
    pub fn enable_fifo(self) -> Lsm6dsv32<'d, FifoEnabled, I1, I2> {
        Lsm6dsv32 {
            hw: self.hw,
            config: ImuConfig {
                fifo_state: crate::lsm6dsv32_config::FifoEnabled,
                int1_state: self.config.int1_state,
                int2_state: self.config.int2_state,
                general: self.config.general,
                accel: self.config.accel,
                gyro: self.config.gyro,
                fifo: self.config.fifo,
                int1: self.config.int1,
                int2: self.config.int2,
            },
        }
    }
    pub fn config_fifo(
        mut self,
        mode: FIFOMode,
        gyro: GyroBatchDataRate,
        accel: AccelBatchDataRate,
        temp: TempatureBatchRate,
        ts: TimeStampBatch,
    ) {
        let config_old = &self.config.fifo;
        self.config.fifo = FifoConfig {
            mode,
            gyro_fifo: gyro,
            accel_fifo: accel,
            temp_fifo: temp,
            ts_fifo: ts,
            watermark_threshold: config_old.watermark_threshold,
            counter_trigger: config_old.counter_trigger,
            counter_threshold: config_old.counter_threshold,
        };
    }
    pub fn set_watermark_threshold(&mut self, threshold: u8) {
        self.config.fifo.watermark_threshold = threshold;
    }

    pub fn set_counter(&mut self, trigger: TriggerCounter, threshold: u16) {
        self.config.fifo.counter_trigger = trigger;
        self.config.fifo.counter_threshold = threshold;
    }
}

/*
impl<I1,I2> ImuConfig<FifoDisabled, I1, I2> {
    pub fn enable_fifo(self, fifo_config: FifoConfig) -> ImuConfig<FifoEnabled, I1, I2> {
        ImuConfig {
            fifo_state: FifoEnabled,
            int1_state: self.int1_state,
            int2_state: self.int2_state,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: fifo_config,
            int1: self.int1,
            int2: self.int2,
        }
    }

}

impl<I1,I2> ImuConfig<FifoEnabled, I1, I2> {
    pub fn disable_fifo(self) -> ImuConfig<FifoDisabled, I1, I2> {
        ImuConfig {
            fifo_state: FifoDisabled,
            int1_state: self.int1_state,
            int2_state: self.int2_state,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: self.int1,
            int2: self.int2,
        }
    }
}

impl<F,I2> ImuConfig<F,Int1Disabled, I2> {
    pub fn enable_int1(self, int1_config: Interrupt1Config) -> ImuConfig<F, Int1Enabled, I2> {
        ImuConfig {
            fifo_state: self.fifo_state,
            int1_state: Int1Enabled,
            int2_state: self.int2_state,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: int1_config,
            int2: self.int2,
        }
    }
}

impl<F,I2> ImuConfig<F,Int1Enabled, I2> {
    pub fn disable_int1(self) -> ImuConfig<F, Int1Disabled, I2> {
        ImuConfig {
            fifo_state: self.fifo_state,
            int1_state: Int1Disabled,
            int2_state: self.int2_state,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: self.int1,
            int2: self.int2,
        }
    }
}

impl<F,I1> ImuConfig<F,I1,Int2Disabled> {
    pub fn enable_int2(self, int2_config: Interrupt2Config) -> ImuConfig<F, I1, Int2Enabled> {
        ImuConfig {
            fifo_state: self.fifo_state,
            int1_state: self.int1_state,
            int2_state: Int2Enabled,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: self.int1,
            int2: int2_config,
        }
    }
}

impl<F,I1> ImuConfig<F,I1,Int2Enabled> {
    pub fn disable_int2(self) -> ImuConfig<F, I1, Int2Disabled> {
        ImuConfig {
            fifo_state: self.fifo_state,
            int1_state: self.int1_state,
            int2_state: Int2Disabled,
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: self.int1,
            int2: self.int2,
        }
    }
}

*/
