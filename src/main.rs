use anyhow::Context;
use bme_680::{Bme680, Config, OverSample, WaitFactor};

fn main() -> anyhow::Result<()> {
    let loop_time = std::time::Duration::from_secs(2);
    let mut config = Config::new();
    config.humidity(OverSample::X1);
    config.temperature(OverSample::X1);
    config.pressure(OverSample::X1);
    config.heater_on_time(25, WaitFactor::F4)?;
    config.heater_temp(300);
    config.run_gas(true);
    let bme = Bme680::open(config)?.apply_settings()?;
    println!("Config:\n{:?}", config);
    loop {
        let now = std::time::Instant::now();
        match bme
            .sample_with_delay(150)
            .context("error during sample - retrying")
        {
            Ok(data) => {
                println!(
                    "{:>width$.1}hPa {:>width$.2}C {:>width$.2}% {:>width$.2}",
                    data.pressure / 100.0,
                    data.temperature,
                    data.humidity,
                    data.gas.unwrap_or(0.0),
                    width = 8
                );
            }
            Err(e) => eprintln!("{:?}", e),
        }
        let elapsed = now.elapsed();
        if elapsed < loop_time {
            let sleep = loop_time - elapsed;
            std::thread::sleep(sleep);
        }
    }
}
