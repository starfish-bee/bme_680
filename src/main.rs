use i2c::I2c;

struct Bme680 {
    device: I2c,
    config: Config,
}

impl Bme680 {
    fn open(config: Config) -> Result<Self, i2c::I2cError> {
        let device = I2c::open(0x76)?;
        Ok(Self { device, config })
    }

    fn sample(&self) -> Result<(u32, u32, u16), i2c::I2cError> {
        let address_1 = 0x72;
        let address_2 = 0x74;
        let mode = 1;

        let mut buffer = [0; 4];
        buffer[0] = address_1;
        buffer[1] = u8::from(self.config.h_oversample);
        buffer[2] = address_2;
        buffer[3] = (u8::from(self.config.t_oversample) << 5)
            | (u8::from(self.config.p_oversample) << 2)
            | mode;

        let mut messages = self.device.i2c_buffer();
        messages.add_write(0, &buffer);
        messages.execute()?;

        std::thread::sleep(std::time::Duration::from_secs(1));
        println!("{:x?}", self.device.i2c_read(0x72, 3));
        self.get_readings()
    }

    fn get_readings(&self) -> Result<(u32, u32, u16), i2c::I2cError> {
        let address = 0x1F;
        let buffer = self.device.i2c_read(address, 8)?;

        let pressure = (u32::from(buffer[0]) << 12)
            | (u32::from(buffer[1]) << 4)
            | (u32::from(buffer[2]) >> 4);
        let temperature = (u32::from(buffer[3]) << 12)
            | (u32::from(buffer[4]) << 4)
            | (u32::from(buffer[5]) >> 4);
        let humidity = (u16::from(buffer[6]) << 8) | u16::from(buffer[7]);

        Ok((pressure, temperature, humidity))
    }

    fn reset(&self) -> Result<bool, i2c::I2cError> {
        let value = 0xB6;
        let register = 0xE0;
        let buf = [register, value];
        let mut buffer = self.device.i2c_buffer();
        buffer.add_write(0, &buf);
        buffer.execute()?;

        let test_value = 0x80;
        let test_register = 0x1F;
        match self.device.i2c_read(test_register, 1)?[..1] {
            [x] if x == test_value => Ok(true),
            _ => Ok(false),
        }
    }
}

struct Config {
    h_oversample: OverSample,
    t_oversample: OverSample,
    p_oversample: OverSample,
}

impl Config {
    fn new() -> Self {
        Self {
            h_oversample: OverSample::X0,
            t_oversample: OverSample::X0,
            p_oversample: OverSample::X0,
        }
    }

    fn humidity(&mut self, oversample: OverSample) {
        self.h_oversample = oversample
    }
    fn temperature(&mut self, oversample: OverSample) {
        self.t_oversample = oversample
    }
    fn pressure(&mut self, oversample: OverSample) {
        self.p_oversample = oversample
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
enum OverSample {
    X0 = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

impl std::convert::From<OverSample> for u8 {
    fn from(arg: OverSample) -> u8 {
        arg as u8
    }
}

fn main() {
    let mut config = Config::new();
    config.humidity(OverSample::X1);
    config.temperature(OverSample::X2);
    config.pressure(OverSample::X16);
    let bme = Bme680::open(config).unwrap();
    println!("{:?}", bme.sample());
    println!("{:x?}", bme.device.i2c_read(0x72, 3));
    println!("{:x?}", bme.reset());
}
