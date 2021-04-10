use i2c::I2c;
use std::convert::TryInto;

// register addresses taken from BME680 datasheet (page 28)
const REG_CTRL_HUM: u8 = 0x72;
const REG_CTRL_MEAS: u8 = 0x74;
// parameter registers, page 18
const REG_PAR_T1: u8 = 0xE9;
const REG_PAR_T23: u8 = 0x8A;

struct Ready;
struct NotReady;

struct Bme680<T> {
    device: I2c,
    config: Config,
    calibration: CalibrationParameters,
    _phantom: std::marker::PhantomData<T>,
}

impl Bme680<NotReady> {
    fn open(config: Config) -> Result<Self, i2c::I2cError> {
        let device = I2c::open(0x76)?;
        let calibration = CalibrationParameters::read(&device)?;
        Ok(Self {
            device,
            config,
            calibration,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl Bme680<Ready> {
    fn sample(&self) -> Result<(u32, f64, u16), i2c::I2cError> {
        let mode = 1;

        let to_write = [
            REG_CTRL_MEAS,
            (u8::from(self.config.t_oversample) << 5)
                | (u8::from(self.config.p_oversample) << 2)
                | mode,
        ];
        self.device.i2c_buffer().add_write(0, &to_write).execute()?;

        std::thread::sleep(std::time::Duration::from_secs(1));
        let (p, t, h) = self.read_pth()?;
        let t = self.calibration.calculate_temperature(t);
        Ok((p, t, h))
    }

    fn read_pth(&self) -> Result<(u32, u32, u16), i2c::I2cError> {
        // pressure, temperature and humidity registers go from 0x1F to 0x26
        // see page 28, BME680 datasheet
        let address = 0x1F;
        let buffer = self.device.i2c_read(address, 8)?;
        let buf_checked = &buffer[..8];

        // pressure bits - 0x1F, 0x20, first 4 bits of 0x21
        let temp = buf_checked[..4].try_into().unwrap();
        let pressure = u32::from_be_bytes(temp) >> 12;
        // temperature bits - 0x22, 0x23, first 4 bits of 0x24
        let temp = buf_checked[3..7].try_into().unwrap();
        let temperature = u32::from_be_bytes(temp) >> 12;
        // humidity bits - 0x25, 0x26
        let temp = buf_checked[6..8].try_into().unwrap();
        let humidity = u16::from_be_bytes(temp);

        Ok((pressure, temperature, humidity))
    }
}

impl<T> Bme680<T> {
    fn apply_settings(self) -> Result<Bme680<Ready>, i2c::I2cError> {
        let to_write = [REG_CTRL_HUM, u8::from(self.config.h_oversample)];
        self.device.i2c_buffer().add_write(0, &to_write).execute()?;
        Ok(Bme680 {
            device: self.device,
            config: self.config,
            calibration: self.calibration,
            _phantom: std::marker::PhantomData,
        })
    }

    fn reset(self) -> Result<Bme680<NotReady>, i2c::I2cError> {
        // writing 0xB6 to register 0xE0 triggers a soft reset
        // see page 30, BME680 datasheet
        let value = 0xB6;
        let register = 0xE0;
        let buf = [register, value];
        self.device.i2c_buffer().add_write(0, &buf).execute()?;

        // register 0x1F reset state value is 0x80
        // see page 28, BME680 datasheet
        //let test_value = 0x80;
        //let test_register = 0x1F;
        //match self.device.i2c_read(test_register, 1)?[..1] {
        //    [x] if x == test_value => Ok(true),
        //    _ => Ok(false),
        //}
        Ok(Bme680 {
            device: self.device,
            config: self.config,
            calibration: self.calibration,
            _phantom: std::marker::PhantomData,
        })
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

struct CalibrationParameters {
    temp_1: u16,
    temp_2: u16,
    temp_3: u8,
}

impl CalibrationParameters {
    fn read(device: &I2c) -> Result<Self, i2c::I2cError> {
        let mut buffer = [0; 5];
        let (mut t_1, mut t_23) = buffer.split_at_mut(2);

        device
            .i2c_buffer()
            .add_write(0, std::slice::from_ref(&REG_PAR_T1))
            .add_read(0, &mut t_1)
            .execute()?;
        device
            .i2c_buffer()
            .add_write(0, std::slice::from_ref(&REG_PAR_T23))
            .add_read(0, &mut t_23)
            .execute()?;

        let buf_checked = &buffer[..5];

        // par_t1 bits, 0xEA, 0xE9
        let par = buf_checked[..2].try_into().unwrap();
        let temp_1 = u16::from_le_bytes(par);
        // par_t2 bits, 0x8B, 0x8A
        let par = buf_checked[2..4].try_into().unwrap();
        let temp_2 = u16::from_le_bytes(par);
        // par_t3 bits, 0x8C
        let temp_3 = buffer[4];

        Ok(Self {
            temp_1,
            temp_2,
            temp_3,
        })
    }

    // calculation as definted in BME680 datasheet (page 17)
    fn calculate_temperature(&self, raw_temp: u32) -> f64 {
        let raw_temp = f64::from(raw_temp);
        let temp_1 = f64::from(self.temp_1);
        let temp_2 = f64::from(self.temp_2);
        let temp_3 = f64::from(self.temp_3);

        let var_1 = temp_2 * (raw_temp / 16384.0 - temp_1 / 1024.0);
        let var_2 = 16.0
            * temp_3
            * (raw_temp / 131072.0 - temp_1 / 8192.0)
            * (raw_temp / 131072.0 - temp_1 / 8192.0);
        (var_1 + var_2) / 5120.0
    }
}

fn main() {
    let mut config = Config::new();
    config.humidity(OverSample::X1);
    config.temperature(OverSample::X16);
    config.pressure(OverSample::X16);
    let bme = Bme680::open(config).unwrap().apply_settings().unwrap();
    println!("{:?}", bme.sample());
    bme.reset().unwrap();
}
