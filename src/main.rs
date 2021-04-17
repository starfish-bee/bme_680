use bme_680::{Bme680, Config, OverSample};

fn main() {
    let loop_time = std::time::Duration::from_secs(5);
    let mut config = Config::new();
    config.humidity(OverSample::X1);
    config.temperature(OverSample::X2);
    config.pressure(OverSample::X16);
    let bme = Bme680::open(config).unwrap().apply_settings().unwrap();
    println!("Config:\n{:?}", config);
    loop {
        let now = std::time::Instant::now();
        let (p, t, h) = bme.sample().unwrap();
        println!(
            "{:>width$.1}hPa {:>width$.2}C {:>width$.2}%",
            p / 100.0,
            t,
            h,
            width = 8
        );
        let elapsed = now.elapsed();
        if elapsed < loop_time {
            let sleep = loop_time - elapsed;
            std::thread::sleep(sleep);
        }
    }
}
