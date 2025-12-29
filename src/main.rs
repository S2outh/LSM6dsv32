#![no_std]
#![no_main]

use LSM6DSV32::lsm6dsv32::*;
use defmt::*;
use defmt_rtt as _;
use embassy_time::Timer;
use heapless::String;
use panic_probe as _;
use LSM6DSV32::lsm6dsv32_config::*;
use embassy_executor::{Spawner, task};
use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, gpio::{Level, Output, Pull, Speed}, interrupt, mode::Async, pac::usb::vals::Stat, spi::{Config, Instance, Mode, Phase, Polarity, Spi}, time::Hertz, usart::{self, Config as UartConfig, Uart}
};
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

    let int1 = INT1.init(ExtiInput::new(
        p.PB1, 
        p.EXTI1, 
        Pull::Down));
    
    let int2 = INT2.init(ExtiInput::new(
        p.PB2,
        p.EXTI2,
        Pull::Down));

    let mut lsm = Lsm6dsv32::new(spi, cs, int1, int2, false).await;
    let mut lsm = lsm.enable_fifo();
    lsm.config_fifo(FIFOMode::ContinuousMode, GyroBatchDataRate::NotBatched, AccelBatchDataRate::Hz1_875, TempatureBatchRate::NotBatched, TimeStampBatch::Every32);
    lsm.set_counter(TriggerCounter::Gyro,16);
    lsm.commit_config().await;
    
    let scale = lsm.calc_scaling(UnitScale::Default);
   
    loop {
        if let Err(e) = lsm.read_data_when_ready_polling(true, false, false, true, 10, LogicOp::OR, |s_raw| {
            let s = s_raw.create_f32(scale);
            info!("IMU DATA -> Accel: {:?}, Accel2: {:?} Gyro: {:?}, TS: {}, deltaTS: {}", s.accel, s.accel_ch2,s.gyro, s.last_ts, s.delta_ts);
        }).await {
            error!("Fehler");
        }
        /* 
        let status = lsm.read_fifo_status().await.unwrap_or_else(|e|{error!("Fehler beim Lesen {:?}",e); FifoStatusSample::from_registers(0,   0)});
        info!("{}",status.unread_samples)
        
        let result = lsm.read_timestamp().await.unwrap_or_else(|e|{error!("Fehler beim Lesen {:?}",e); 0});
        info!("{}",result);
        Timer::after_millis(500).await;
         
        lsm.fifo_read_data_polling(FifoTrigger::Counter, 10, false,
            |se| {
                
            let s = se.create_f32(scale);
            //info!("IMU DATA -> Accel: {:?}, Accel2: {:?} Gyro: {:?}, TS: {}, deltaTS: {}", s.accel, s.accel_ch2,s.gyro, s.last_ts, s.delta_ts);
            },
            Some(|t| {
            info!("TEMP UPDATE -> {} °C", temp_f32(t));
            })
        ).await.unwrap_or_else(|e| error!("FIFO Error: {:?}", e));   
       */

        

    }
    //lsm.enable_debug(true);

}
