#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use log::*;
use defmt_rtt as _; // global logger
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    info!("Hello World!");
    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("tick");
    }
}
