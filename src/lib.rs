use i2c::{I2c, I2cError};
use std::convert::TryInto;
use thiserror::Error;

// BME680 I2C address
const I2C_ADDRESS: u16 = 0x76;

// register addresses taken from BME680 datasheet (page 28)
const REG_CTRL_HUM: u8 = 0x72;
const REG_CTRL_MEAS: u8 = 0x74;
// temperature parameter registers, page 18
const REG_PAR_T1: u8 = 0xE9;
const REG_PAR_T23: u8 = 0x8A;
// pressure parameter registers, page 19
const REG_PAR_P: u8 = 0x8E;
// humidity parameter registers, page 20
const REG_PAR_H: u8 = 0xE1;

pub type BmeResult<T> = Result<T, BmeError>;

pub struct Ready;
pub struct NotReady;

#[derive(Debug)]
pub struct Bme680<T> {
    pub device: I2c,
    config: Config,
    calibration: CalibrationParameters,
    _phantom: std::marker::PhantomData<T>,
}

impl Bme680<NotReady> {
    pub fn open(config: Config) -> BmeResult<Self> {
        let device = I2c::open(I2C_ADDRESS).map_err(BmeError::OpenError)?;
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
    pub fn sample(&self) -> BmeResult<(f64, f64, f64)> {
        let mode = 1;

        let to_write = [
            REG_CTRL_MEAS,
            (u8::from(self.config.t_oversample) << 5)
                | (u8::from(self.config.p_oversample) << 2)
                | mode,
        ];
        self.device
            .i2c_buffer()
            .add_write(0, &to_write)
            .execute()
            .map_err(BmeError::ForceError)?;

        std::thread::sleep(std::time::Duration::from_secs(1));
        let (p, t, h) = self.read_pth()?;
        let (p, t, h) = self.calibration.calculate_pth(p, t, h);
        Ok((p, t, h))
    }

    fn read_pth(&self) -> BmeResult<(i32, i32, i16)> {
        // pressure, temperature and humidity registers go from 0x1F to 0x26
        // see page 28, BME680 datasheet
        let address = 0x1F;
        let mut buffer = [0; 8];
        self.device
            .i2c_read(address, &mut buffer)
            .map_err(BmeError::MeasurementError)?;
        let buf_checked = &buffer[..8];

        // pressure bits - 0x1F, 0x20, first 4 bits of 0x21
        let temp = buf_checked[..4].try_into().unwrap();
        let pressure = i32::from_be_bytes(temp) >> 12;
        // temperature bits - 0x22, 0x23, first 4 bits of 0x24
        let temp = buf_checked[3..7].try_into().unwrap();
        let temperature = i32::from_be_bytes(temp) >> 12;
        // humidity bits - 0x25, 0x26
        let temp = buf_checked[6..8].try_into().unwrap();
        let humidity = i16::from_be_bytes(temp);

        Ok((pressure, temperature, humidity))
    }
}

impl<T> Bme680<T> {
    pub fn apply_settings(self) -> BmeResult<Bme680<Ready>> {
        let to_write = [REG_CTRL_HUM, u8::from(self.config.h_oversample)];
        self.device
            .i2c_buffer()
            .add_write(0, &to_write)
            .execute()
            .map_err(BmeError::SetError)?;
        Ok(Bme680 {
            device: self.device,
            config: self.config,
            calibration: self.calibration,
            _phantom: std::marker::PhantomData,
        })
    }

    pub fn reset(self) -> BmeResult<Bme680<NotReady>> {
        // writing 0xB6 to register 0xE0 triggers a soft reset
        // see page 30, BME680 datasheet
        let value = 0xB6;
        let register = 0xE0;
        let buf = [register, value];
        self.device
            .i2c_buffer()
            .add_write(0, &buf)
            .execute()
            .map_err(BmeError::SetError)?;

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

    pub fn get_config(&self) -> &Config {
        &self.config
    }

    pub fn get_calibration(&self) -> &CalibrationParameters {
        &self.calibration
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Config {
    h_oversample: OverSample,
    t_oversample: OverSample,
    p_oversample: OverSample,
}

impl Config {
    pub fn new() -> Self {
        Self {
            h_oversample: OverSample::X0,
            t_oversample: OverSample::X0,
            p_oversample: OverSample::X0,
        }
    }

    pub fn humidity(&mut self, oversample: OverSample) {
        self.h_oversample = oversample
    }

    pub fn temperature(&mut self, oversample: OverSample) {
        self.t_oversample = oversample
    }

    pub fn pressure(&mut self, oversample: OverSample) {
        self.p_oversample = oversample
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum OverSample {
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

// all parameter types taken from:
// https://github.com/BoschSensortec/BME68x-Sensor-API/blob/master/bme68x_defs.h
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct CalibrationParameters {
    temperature: TempParams,
    pressure: PresParams,
    humidity: HumParams,
}

// see datasheet page 18 for addressess
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct TempParams {
    temp_1: u16,
    temp_2: u16,
    temp_3: i8,
}

// see datasheet page 19 for addressess
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct PresParams {
    pres_1: u16,
    pres_2: i16,
    pres_3: i8,
    pres_4: i16,
    pres_5: i16,
    pres_6: i8,
    pres_7: i8,
    pres_8: i16,
    pres_9: i16,
    pres_10: u8,
}

// see datasheet page 20 for addressess
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct HumParams {
    hum_1: u16,
    hum_2: u16,
    hum_3: i8,
    hum_4: i8,
    hum_5: i8,
    hum_6: u8,
    hum_7: i8,
}

impl CalibrationParameters {
    fn read(device: &I2c) -> BmeResult<Self> {
        let temperature = CalibrationParameters::read_temp_params(device)?;
        let pressure = CalibrationParameters::read_pres_params(device)?;
        let humidity = CalibrationParameters::read_hum_params(device)?;

        Ok(Self {
            temperature,
            pressure,
            humidity,
        })
    }

    fn calculate_pth(&self, raw_pres: i32, raw_temp: i32, raw_hum: i16) -> (f64, f64, f64) {
        let (temperature, t_fine) = self.calculate_temperature(raw_temp);
        let pressure = self.calculate_pressure(raw_pres, t_fine);
        let humidity = self.calculate_humidity(raw_hum, temperature);

        (pressure, temperature, humidity)
    }

    fn read_temp_params(device: &I2c) -> BmeResult<TempParams> {
        let mut buffer = [0; 5];
        let (mut t_1, mut t_23) = buffer.split_at_mut(2);

        device
            .i2c_read(REG_PAR_T1, &mut t_1)
            .map_err(BmeError::ParamError)?;
        device
            .i2c_read(REG_PAR_T23, &mut t_23)
            .map_err(BmeError::ParamError)?;

        let buf_checked = &buffer[..5];

        // par_t1 bits, 0xEA, 0xE9
        let par = buf_checked[..2].try_into().unwrap();
        let temp_1 = u16::from_le_bytes(par);
        // par_t2 bits, 0x8B, 0x8A
        let par = buf_checked[2..4].try_into().unwrap();
        let temp_2 = u16::from_le_bytes(par);
        // par_t3 bits, 0x8C
        let par = buf_checked[4..5].try_into().unwrap();
        let temp_3 = i8::from_le_bytes(par);

        Ok(TempParams {
            temp_1,
            temp_2,
            temp_3,
        })
    }

    fn read_pres_params(device: &I2c) -> BmeResult<PresParams> {
        let mut buffer = [0; 19];
        device
            .i2c_read(REG_PAR_P, &mut buffer)
            .map_err(BmeError::ParamError)?;
        let buf_checked = &buffer[..19];

        // par_p1 bits, 0x8F, 0x8E
        let par = buf_checked[..2].try_into().unwrap();
        let pres_1 = u16::from_le_bytes(par);
        // par_p2 bits, 0x91, 0x90
        let par = buf_checked[2..4].try_into().unwrap();
        let pres_2 = i16::from_le_bytes(par);
        // par_p3 bits, 0x92
        let par = buf_checked[4..5].try_into().unwrap();
        let pres_3 = i8::from_le_bytes(par);
        // par_p4 bits, 0x95, 0x94
        let par = buf_checked[6..8].try_into().unwrap();
        let pres_4 = i16::from_le_bytes(par);
        // par_p5 bits, 0x97, 0x96
        let par = buf_checked[8..10].try_into().unwrap();
        let pres_5 = i16::from_le_bytes(par);
        // par_p6 bits, 0x99
        let par = buf_checked[11..12].try_into().unwrap();
        let pres_6 = i8::from_le_bytes(par);
        // par_p7 bits, 0x98
        let par = buf_checked[10..11].try_into().unwrap();
        let pres_7 = i8::from_le_bytes(par);
        // par_p8 bits, 0x9D, 0x9C
        let par = buf_checked[14..16].try_into().unwrap();
        let pres_8 = i16::from_le_bytes(par);
        // par_p9 bits, 0x9F, 0x9E
        let par = buf_checked[16..18].try_into().unwrap();
        let pres_9 = i16::from_le_bytes(par);
        // par_p10 bits, 0xA0
        let par = buf_checked[18..19].try_into().unwrap();
        let pres_10 = u8::from_le_bytes(par);

        Ok(PresParams {
            pres_1,
            pres_2,
            pres_3,
            pres_4,
            pres_5,
            pres_6,
            pres_7,
            pres_8,
            pres_9,
            pres_10,
        })
    }

    fn read_hum_params(device: &I2c) -> BmeResult<HumParams> {
        let mut buffer = [0; 8];
        device
            .i2c_read(REG_PAR_H, &mut buffer)
            .map_err(BmeError::ParamError)?;
        let buf_checked = &buffer[..8];

        // par_p1 bits, 0xE3, 0xE2<3:0>
        let mut par: [u8; 2] = buf_checked[1..3].try_into().unwrap();
        par[0] <<= 4;
        let hum_1 = u16::from_le_bytes(par) >> 4;
        // par_p2 bits, 0xE1, 0xE2<7:4>
        let par = buf_checked[..2].try_into().unwrap();
        let hum_2 = u16::from_be_bytes(par) >> 4;
        // par_p3 bits, 0xE4
        let par = buf_checked[3..4].try_into().unwrap();
        let hum_3 = i8::from_le_bytes(par);
        // par_p4 bits, 0xE5
        let par = buf_checked[4..5].try_into().unwrap();
        let hum_4 = i8::from_le_bytes(par);
        // par_p5 bits, 0xE6
        let par = buf_checked[5..6].try_into().unwrap();
        let hum_5 = i8::from_le_bytes(par);
        // par_p6 bits, 0xE7
        let par = buf_checked[6..7].try_into().unwrap();
        let hum_6 = u8::from_le_bytes(par);
        // par_p7 bits, 0xE8
        let par = buf_checked[7..8].try_into().unwrap();
        let hum_7 = i8::from_le_bytes(par);

        Ok(HumParams {
            hum_1,
            hum_2,
            hum_3,
            hum_4,
            hum_5,
            hum_6,
            hum_7,
        })
    }

    // calculation as definted in BME680 datasheet (page 17)
    // t_fine parameter returned for use in pressure calculation (see page 18)
    fn calculate_temperature(&self, raw_temp: i32) -> (f64, f64) {
        const PAR1: f64 = 1.0 / 16384.0;
        const PAR2: f64 = 1.0 / 1024.0;
        const PAR3: f64 = 1.0 / 131072.0;
        const PAR4: f64 = 1.0 / 8192.0;
        const PAR5: f64 = 1.0 / 5120.0;

        let raw_temp = f64::from(raw_temp);
        let params = &self.temperature;
        let temp_1 = f64::from(params.temp_1);
        let temp_2 = f64::from(params.temp_2);
        let temp_3 = f64::from(params.temp_3);

        let var_1 = temp_2 * (raw_temp * PAR1 - temp_1 * PAR2);
        let var_2 =
            16.0 * temp_3 * (raw_temp * PAR3 - temp_1 * PAR4) * (raw_temp * PAR3 - temp_1 * PAR4);
        let t_fine = var_1 + var_2;
        (t_fine * PAR5, t_fine)
    }

    // calculation as definted in BME680 datasheet (page 18)
    fn calculate_pressure(&self, raw_pres: i32, t_fine: f64) -> f64 {
        const PAR1: f64 = 1.0 / 131072.0;
        const PAR2: f64 = 1.0 / 16384.0;
        const PAR3: f64 = 1.0 / 524288.0;
        const PAR4: f64 = 1.0 / 32768.0;
        const PAR5: f64 = 1.0 / 4096.0;
        const PAR6: f64 = 1.0 / 2147483648.0;
        const PAR7: f64 = 1.0 / 256.0;
        const PAR8: f64 = 1.0 / 16.0;

        let raw_pres = f64::from(raw_pres);
        let params = &self.pressure;
        let pres_1 = f64::from(params.pres_1);
        let pres_2 = f64::from(params.pres_2);
        let pres_3 = f64::from(params.pres_3);
        let pres_4 = f64::from(params.pres_4);
        let pres_5 = f64::from(params.pres_5);
        let pres_6 = f64::from(params.pres_6);
        let pres_7 = f64::from(params.pres_7);
        let pres_8 = f64::from(params.pres_8);
        let pres_9 = f64::from(params.pres_9);
        let pres_10 = f64::from(params.pres_10);

        let var_1 = t_fine * 0.5 - 64000.0;
        let var_2 = var_1 * var_1 * pres_6 * PAR1;
        let var_2 = var_2 + var_1 * pres_5 * 2.0;
        let var_2 = var_2 * 0.25 + pres_4 * 65536.0;
        let var_1 = (var_1 * var_1 * pres_3 * PAR2 + var_1 * pres_2) * PAR3;
        let var_1 = (1.0 + var_1 * PAR4) * pres_1;
        let pressure = 1048576.0 - raw_pres;
        let pressure = (pressure - var_2 * PAR5) * 6250.0 / var_1;
        let var_1 = pressure * pressure * pres_9 * PAR6;
        let var_2 = pressure * pres_8 * PAR4;
        let var_3 = pressure * pressure * pressure * pres_10 * PAR7 * PAR7 * PAR7 * PAR1;
        pressure + (var_1 + var_2 + var_3 + pres_7 * 128.0) * PAR8
    }

    // calculation as definted in BME680 datasheet (page 20)
    fn calculate_humidity(&self, raw_hum: i16, temp: f64) -> f64 {
        const PAR1: f64 = 1.0 / 262144.0;
        const PAR2: f64 = 1.0 / 16384.0;
        const PAR3: f64 = 1.0 / 1048576.0;
        const PAR4: f64 = 1.0 / 2097152.0;

        let raw_hum = f64::from(raw_hum);
        let params = &self.humidity;
        let hum_1 = f64::from(params.hum_1);
        let hum_2 = f64::from(params.hum_2);
        let hum_3 = f64::from(params.hum_3);
        let hum_4 = f64::from(params.hum_4);
        let hum_5 = f64::from(params.hum_5);
        let hum_6 = f64::from(params.hum_6);
        let hum_7 = f64::from(params.hum_7);

        let var_1 = raw_hum - (hum_1 * 16.0 + temp * hum_3 * 0.5);
        let var_2 = var_1 * hum_2 * PAR1 * (1.0 + hum_4 * PAR2 * temp + hum_5 * PAR3 * temp * temp);
        let var_3 = hum_6 * PAR2;
        let var_4 = hum_7 * PAR4;
        var_2 + (var_3 + var_4 * temp) * var_2 * var_2
    }
}

#[derive(Debug, Error)]
pub enum BmeError {
    #[error("failed to open device at address {}", I2C_ADDRESS)]
    OpenError(#[source] I2cError),
    #[error("failed to set force mode")]
    ForceError(#[source] I2cError),
    #[error("failed to read measurements")]
    MeasurementError(#[source] I2cError),
    #[error("failed to read calibration parameters")]
    ParamError(#[source] I2cError),
    #[error("failed to set force mode")]
    SetError(#[source] I2cError),
    #[error("failed to set force mode")]
    ResetError(#[source] I2cError),
}
