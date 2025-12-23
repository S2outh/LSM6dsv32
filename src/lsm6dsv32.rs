#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;

use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Speed},
    mode::Async,
    spi::{Config, Instance, Mode, Phase, Polarity, Spi},
    time::Hertz,
};

use crate::lsm6dsv32_config::ImuConfig;


type SpiError = embassy_stm32::spi::Error;

const PIN_CTRL: u8 = 0x02;
const IF_CTRL: u8 = 0x03;
const FIFO_CTRL1: u8 = 0x07;
const FIFO_CTRL3: u8 = 0x09;
const FIFO_CTRL4: u8 = 0x0A;
const COUNTER_BDR_REG1: u8 = 0x0B;
const COUNTER_BDR_REG2: u8 = 0x0C;
const WHO_AM_I: u8 = 0x0F;
const INT1_CTRL: u8 = 0x0D;
const INT2_CTRL: u8 = 0x0E;
const CTRL1: u8 = 0x10;
const CTRL2: u8 = 0x11;
const CTRL4: u8 = 0x13;
const CTRL6: u8 = 0x15;
const CTRL7: u8 = 0x16;
const CTRL8: u8 = 0x17;
const CTRL9: u8 = 0x18;
const OUT_TEMP_L: u8 = 0x20;
const OUTX_L_G: u8 = 0x22;
const UI_OUTX_L_G_OIS_EIS: u8 = 0x2E;
const UI_OUTX_L_A_OIS_DUAL_C: u8 = 0x34;
const TIMESTAMP0: u8 = 0x40;
const MD1_CFG: u8 = 0x5E;
const MD2_CFG: u8 = 0x5F;
const FIFO_DATA_OUT_TAG: u8 = 0x78;
const FIFO_DATA_OUT_X_L: u8 = 0x79;
const EXPECTED_WHO_AM_I: u8 = 0x70;

#[derive(defmt::Format)]
pub enum Error {
    Spi(SpiError),
    NoValue,
    WrongWhoAmI(u8),
}

#[macro_export]
macro_rules! reg8 {
    (
        $(#[$meta:meta])*
        struct $name:ident {
            $($field:ident : $fty:ty => $shift:literal , $width:literal),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        pub struct $name {
            pub addr: u8,
            $(pub $field: $fty),*
        }

        impl Default for $name {
            fn default() -> Self {
                Self {
                    addr: 0,
                    $($field: 0 as $fty),*
                }
            }
        }

        impl $name {


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
            reg8!($(#[$meta])* struct $name  { $($field : $fty => $shift, $width),* });
            let mut r = $name::default();
            r.addr = $addr;
            $(
                r.$field = $value as $fty;
            )*
            r
        }
    };
}

macro_rules! cfg_bits {
    (
        ($old_value:expr) {
            $($bit:literal => $value:expr),* $(,)?
        }
    ) => {
        {
            let mut reg = $old_value;

            $(
                reg &= !(1<<$bit);
                reg |= ($value as u8 & 1) << $bit;
            )*

            reg
        }
    };
    (
        ($old_value:expr) {
            $($mode:ident => $value:expr),* $(,)?
        }
    ) => {
        {
            let mut reg_acc = $old_value;

            $(
                match stringify!($mode) {
                    "set" => reg_acc |= $mask,
                    "clear" => reg_acc &= !$mask,
                    _ => {},
                }
            )*

            reg_acc
        }
    };
}

pub struct Lsm6dsv32<'d> {
    spi: &'d mut Spi<'d, Async>,
    cs: &'d mut Output<'d>,
    int1: &'d mut ExtiInput<'d>,
    int2: &'d mut ExtiInput<'d>,
    accel_fs: f32,
    gyro_fs: f32,
    config: Option<ImuConfig>,
    bias_accel: [f32; 3],
    bias_gyro: [f32; 3],
}

impl<'d> Lsm6dsv32<'d> {
    pub fn new(
        spi: &'d mut Spi<'d, Async>,
        cs: &'d mut Output<'d>,
        int1: &'d mut ExtiInput<'d>,
        int2: &'d mut ExtiInput<'d>,
    ) -> Self {
        Self {
            spi,
            cs,
            int1,
            int2,
            accel_fs: 0.0,
            gyro_fs: 0.0,
            config: None,
            bias_accel: [0.0; 3],
            bias_gyro: [0.0; 3],
        }
    }

    pub async fn set_config(&mut self, imu_config: ImuConfig) {
        if let Err(e) = self.write_config_registers(&imu_config).await {
            error!("Error writing config registers: {:?}", e);
        } else {
            self.config = Some(imu_config);
        }

        if let Err(e) = self.check_who_i_am().await {
            error!("Error checking WHO_AM_I register: {:?}", e);
        }

        if let Some(cfg) = &self.config {
            self.accel_fs = cfg.accel.calc_scaling_factor();
            self.gyro_fs = cfg.gyro.calc_scaling_factor();
        }
    }

    async fn check_who_i_am(&mut self) -> Result<(), Error> {
        let who_am_i_data = self.read_register(WHO_AM_I).await?;
        if who_am_i_data != EXPECTED_WHO_AM_I {
            return Err(Error::WrongWhoAmI(who_am_i_data));
        }
        Ok(())
    }

    async fn write_config_registers(&mut self, imu_config: &ImuConfig) -> Result<(), Error> {
        let pin_ctrl_old = self.read_register(PIN_CTRL).await?;
        let pin_ctrl = cfg_bits!(
            (pin_ctrl_old) {
                6 => imu_config.general.sdo_pull_up
            }
        );
        self.write_register(PIN_CTRL, pin_ctrl).await?;

        let if_ctrl_old = self.read_register(IF_CTRL).await?;
        let if_ctrl = cfg_bits!(
            (if_ctrl_old) {
                7 => imu_config.general.sda_pull_up,
                5 => imu_config.general.anti_spike_filter,
                4 => imu_config.general.interrupt_lvl,
                3 => imu_config.general.interrupt_pin_mode,
                0 => 1
            }
        );
        self.write_register(IF_CTRL, if_ctrl).await?;

        self.write_register(FIFO_CTRL1, imu_config.fifo.watermark_threshold)
            .await?;

        let fifo_ctrl3 = reg8!(
            FifoCtrl3 (FIFO_CTRL3) {
                gyro_bdr: u8 => 4,4 = imu_config.fifo.gyro_fifo,
                accel_bdr: u8 => 0,4 = imu_config.fifo.accel_fifo,
            }
        );
        self.write_register(fifo_ctrl3.addr, fifo_ctrl3.encode())
            .await?;

        let fifo_ctrl4 = reg8!(
            FifoCtrl4 (FIFO_CTRL4) {
                ts_batch: u8 => 6,2 = imu_config.fifo.ts_fifo,
                temp_batch: u8 => 4,2 = imu_config.fifo.temp_fifo,
                fifo_mode: u8 => 0,3 = imu_config.fifo.mode,
            }
        );
        self.write_register(fifo_ctrl4.addr, fifo_ctrl4.encode())
            .await?;

        let counter_th_89 = ((imu_config.fifo.counter_threshold >> 8) & 0b11) as u8;
        let counter_th_07 = (imu_config.fifo.counter_threshold & 0xFF) as u8;

        let counter_bdr_reg1 = reg8! (
            CounterBdrReg1 (COUNTER_BDR_REG1) {
                trigger: u8 => 5,2 = imu_config.fifo.counter_trigger,
                counter_bdr_th_8_9: u8 => 0,2 = counter_th_89,
            }
        );
        self.write_register(counter_bdr_reg1.addr, counter_bdr_reg1.encode())
            .await?;
        self.write_register(COUNTER_BDR_REG2, counter_th_07).await?;

        let int1_ctrl = cfg_bits!(
            (0u8) {
                6 => imu_config.int1.counter_bdr_int,
                5 => imu_config.int1.fifo_full_int,
                4 => imu_config.int1.fifo_overrun_int,
                3 => imu_config.int1.fifo_threshold_int,
                1 => imu_config.int1.data_ready_gyro,
                0 => imu_config.int1.data_ready_accel
            }
        );
        self.write_register(INT1_CTRL, int1_ctrl).await?;

        let int2_ctrl = cfg_bits!(
            (0u8) {
                6 => imu_config.int2.counter_bdr_int,
                5 => imu_config.int2.fifo_full_int,
                4 => imu_config.int2.fifo_overrun_int,
                3 => imu_config.int2.fifo_threshold_int,
                1 => imu_config.int2.data_ready_gyro,
                0 => imu_config.int2.data_ready_accel
            }
        );
        self.write_register(INT2_CTRL, int2_ctrl).await?;

        let ctrl1 = reg8!(
            Ctrl1 (CTRL1) {
                mode: u8 => 4,3 = imu_config.accel.mode,
                odr: u8 => 0,4 = imu_config.accel.odr,
            }
        );
        self.write_register(ctrl1.addr, ctrl1.encode()).await?;

        let ctrl2 = reg8!(
            Ctrl2 (CTRL2) {
                mode: u8 => 4,3 = imu_config.gyro.mode,
                odr: u8 => 0,4 = imu_config.gyro.odr,
            }
        );
        self.write_register(ctrl2.addr, ctrl2.encode()).await?;

        let ctrl4_old = self.read_register(CTRL4).await?;
        let ctrl4 = cfg_bits!(
            (ctrl4_old) {
                2 => imu_config.int2.temp_ready
            }
        );
        self.write_register(CTRL4, ctrl4).await?;

        let ctrl6 = reg8!(
            Ctrl6 (CTRL6) {
                gyro_lpf1:u8 => 4,3 = imu_config.gyro.lpf1,
                gyro_fs:u8 => 0,4 = imu_config.gyro.full_scale,
            }
        );
        self.write_register(ctrl6.addr, ctrl6.encode()).await?;

        let ctrl7 = imu_config.gyro.lpf1_enabled as u8;
        self.write_register(CTRL7, ctrl7).await?;

        let ctrl8 = reg8!(
            Ctrl8 (CTRL8) {
                lp_hp_f2:u8 => 5,3 = imu_config.accel.lp_hp_f2,
                dual_channel:u8 => 3,1 = imu_config.accel.dual_channel,
                accel_fs:u8 => 0,2 = imu_config.accel.full_scale,
            }
        );
        self.write_register(ctrl8.addr, ctrl8.encode()).await?;

        let ctrl9_old = self.read_register(CTRL9).await?;
        let ctrl9 = cfg_bits!(
            (ctrl9_old) {
                6 => imu_config.accel.hp_reference_mode,
                4 => imu_config.accel.lp_hp,
                3 => imu_config.accel.lpf2_enabled,
                1 => imu_config.accel.user_offset_weight,
                0 => imu_config.accel.user_offset_en
            }
        );
        self.write_register(CTRL9, ctrl9).await?;

        let ofs_usr_x = imu_config.accel.user_offset[0] as u8;
        let ofs_usr_y = imu_config.accel.user_offset[1] as u8;
        let ofs_usr_z = imu_config.accel.user_offset[2] as u8;
        self.write_register(0x73, ofs_usr_x).await?;
        self.write_register(0x74, ofs_usr_y).await?;
        self.write_register(0x75, ofs_usr_z).await?;
        Ok(())
    }

    async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg & !READ_BIT; //Bit 7 = 0 -> Write-Mode -> 0x80: 1000 0000 -> !0x80: 0111 1111

        let mut buffer = [cmd, val];
        self.cs.set_low();
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.cs.set_high();

        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg | READ_BIT;

        let mut buffer = [cmd, 0x00];

        self.cs.set_low();
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.cs.set_high();

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

        self.cs.set_low();
        self.spi
            .transfer_in_place(&mut buffer[..n])
            .await
            .map_err(Error::Spi)?;
        self.cs.set_high();

        data.copy_from_slice(&buffer[1..n]);
        Ok(())
    }
}

