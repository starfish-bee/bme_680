use i2c::{I2c, I2cError};
use std::convert::TryInto;
use thiserror::Error;

// BME680 I2C address
const I2C_ADDRESS: u16 = 0x76;

// register addresses taken from BME680 datasheet memory map (page 28)
// pressure, temperature and humidity data registers
const REG_DATA_PTH: u8 = 0x1F;
// gas data registers
const REG_DATA_G: u8 = 0x2A;
// status registers
const REG_MEAS_STATUS_0: u8 = 0x1D;

// target resistance set point
const REG_RES_HEAT: u8 = 0x5A;
// heater time set point
const REG_GAS_WAIT: u8 = 0x64;
// gas and heater set point register
const REG_CTRL_GAS_1: u8 = 0x71;
// humidity oversample register
const REG_CTRL_HUM: u8 = 0x72;
// temperature and pressure oversample register
const REG_CTRL_MEAS: u8 = 0x74;
// IIR filter register
const REG_CONFIG: u8 = 0x75;

// temperature parameter registers (page 18)
const REG_PAR_T1: u8 = 0xE9;
const REG_PAR_T23: u8 = 0x8A;
// pressure parameter registers (page 19)
const REG_PAR_P: u8 = 0x8E;
// humidity parameter registers (page 20)
const REG_PAR_H: u8 = 0xE1;
// gas parameter registers (page 22)
const REG_PAR_G: u8 = 0xEB;
// heater resistance range register (page 22)
const REG_RES_HEAT_RANGE: u8 = 0x02;
// heater resistance value register (page 22)
const REG_RES_HEAT_VAL: u8 = 0x00;
// range switching error parameter register (page 23)
const REG_RANGE_SWITCHING_ERROR: u8 = 0x04;
//gas constant arrays (page23)
const GAS_ARRAY_1: [f64; 16] = [
    1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992, 1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0,
];
const GAS_ARRAY_2: [f64; 16] = [
    8000000.0,
    4000000.0,
    2000000.0,
    1000000.0,
    499500.4995,
    248262.1648,
    125000.0,
    63004.03226,
    31281.28128,
    15625.0,
    7812.5,
    3906.25,
    1953.125,
    976.5625,
    488.28125,
    244.140625,
];

pub type BmeResult<T> = Result<T, BmeError>;

#[derive(Debug)]
pub struct Ready;
#[derive(Debug)]
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
    pub fn sample(&self) -> BmeResult<Data> {
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

        let mut new_data = 0;
        while (new_data & 0b1000_0000) == 0 {
            self.device
                .i2c_read(REG_MEAS_STATUS_0, std::slice::from_mut(&mut new_data))
                .map_err(BmeError::StatusError)?;
        }

        let data = self.read_pthg()?;
        let data = self.calibration.calculate_values(data);
        Ok(data)
    }

    pub fn sample_with_delay(&self, delay: u64) -> BmeResult<Data> {
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

        let mut new_data = 0;

        std::thread::sleep(std::time::Duration::from_millis(delay));

        while (new_data & 0b1000_0000) == 0 {
            self.device
                .i2c_read(REG_MEAS_STATUS_0, std::slice::from_mut(&mut new_data))
                .map_err(BmeError::StatusError)?;
        }

        let data = self.read_pthg()?;
        let data = self.calibration.calculate_values(data);
        Ok(data)
    }

    fn read_pthg(&self) -> BmeResult<RawData> {
        // pressure, temperature and humidity registers go from 0x1F to 0x26
        // see page 28, BME680 datasheet
        let mut buffer = [0; 8];
        self.device
            .i2c_read(REG_DATA_PTH, &mut buffer)
            .map_err(BmeError::MeasurementError)?;
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

        // gas registers go from 0x2A to 0x2B
        self.device
            .i2c_read(REG_DATA_G, &mut buffer[..2])
            .map_err(BmeError::MeasurementError)?;
        let buf_checked = &buffer[..2];

        // gas bits - 0x2A, first 2 bits of 0x2B
        let temp = buf_checked[..2].try_into().unwrap();
        let resistance = u16::from_be_bytes(temp) >> 6;
        let temp = buf_checked[1..2].try_into().unwrap();
        let status = u8::from_be_bytes(temp);
        let heater_stability = (status & 0b0001_0000) > 0;
        let gas_range = status & 0b0000_0111;

        Ok(RawData {
            pressure,
            temperature,
            humidity,
            gas: (resistance, heater_stability, gas_range),
        })
    }
}

impl<T> Bme680<T> {
    pub fn apply_settings(self) -> BmeResult<Bme680<Ready>> {
        // temperature and pressure oversampling is set in the same register that triggers force
        // mode, so are not set until a measurement is triggered
        let to_write = [
            REG_RES_HEAT,
            self.calibration
                .calculate_heat_res(self.config.heater_temp, 20),
            REG_GAS_WAIT,
            self.config.heater_on_time,
            REG_CTRL_GAS_1,
            u8::from(self.config.run_gas) << 4,
            REG_CTRL_HUM,
            u8::from(self.config.h_oversample),
            REG_CONFIG,
            u8::from(self.config.iir_filter) << 2,
        ];

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
        let test_value = 0x80;
        let test_result = self
            .device
            .i2c_read_bytes(REG_DATA_PTH, 1)
            .map_err(ResetError::I2cError)?[0];
        if test_result != test_value {
            return Err(ResetError::ResetFail.into());
        }

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
struct RawData {
    pressure: u32,
    temperature: u32,
    humidity: u16,
    gas: (u16, bool, u8),
}

#[derive(Debug)]
pub struct Data {
    pub pressure: f64,
    pub temperature: f64,
    pub humidity: f64,
    pub gas: BmeResult<f64>,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Config {
    h_oversample: OverSample,
    t_oversample: OverSample,
    p_oversample: OverSample,
    iir_filter: IirFilter,
    heater_on_time: u8,
    heater_temp: i32,
    run_gas: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    pub fn new() -> Self {
        Self {
            h_oversample: OverSample::X0,
            t_oversample: OverSample::X0,
            p_oversample: OverSample::X0,
            iir_filter: IirFilter::C0,
            heater_on_time: 0,
            heater_temp: 0,
            run_gas: false,
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

    pub fn iir_filter(&mut self, filter: IirFilter) {
        self.iir_filter = filter;
    }

    pub fn heater_on_time(&mut self, time: u8, factor: WaitFactor) -> BmeResult<()> {
        if time > 0b0011_1111 {
            return Err(BmeError::TimeError(time));
        }
        self.heater_on_time = (u8::from(factor) << 6) | time;
        Ok(())
    }

    pub fn heater_temp(&mut self, temp: i32) {
        self.heater_temp = temp;
    }

    pub fn run_gas(&mut self, set: bool) {
        self.run_gas = set;
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum IirFilter {
    C0 = 0b000,
    C1 = 0b001,
    C3 = 0b010,
    C7 = 0b011,
    C15 = 0b100,
    C31 = 0b101,
    C63 = 0b110,
    C127 = 0b111,
}

impl std::convert::From<IirFilter> for u8 {
    fn from(arg: IirFilter) -> u8 {
        arg as u8
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum WaitFactor {
    F1 = 0b000,
    F4 = 0b001,
    F16 = 0b010,
    F64 = 0b011,
}

impl std::convert::From<WaitFactor> for u8 {
    fn from(arg: WaitFactor) -> u8 {
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
    gas: GasParams,
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

// see datasheet page 20 for addressess
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct GasParams {
    gas_1: i8,
    gas_2: i16,
    gas_3: i8,
    heat_range: u8,
    heat_value: i8,
    range_switching_error: i8,
}

impl CalibrationParameters {
    fn read(device: &I2c) -> BmeResult<Self> {
        let temperature = CalibrationParameters::read_temp_params(device)?;
        let pressure = CalibrationParameters::read_pres_params(device)?;
        let humidity = CalibrationParameters::read_hum_params(device)?;
        let gas = CalibrationParameters::read_gas_params(device)?;

        Ok(Self {
            temperature,
            pressure,
            humidity,
            gas,
        })
    }

    fn calculate_values(&self, raw_data: RawData) -> Data {
        let (temperature, t_fine) = self.calculate_temperature(raw_data.temperature);
        let pressure = self.calculate_pressure(raw_data.pressure, t_fine);
        let humidity = self.calculate_humidity(raw_data.humidity, temperature);
        let gas = self.calculate_gas(raw_data.gas);

        Data {
            pressure,
            temperature,
            humidity,
            gas,
        }
    }

    // calculation as definted in BME680 datasheet (page 21)
    fn calculate_heat_res(&self, target_temp: i32, ambient_temp: i32) -> u8 {
        let gas_1 = i32::from(self.gas.gas_1);
        let gas_2 = i32::from(self.gas.gas_2);
        let gas_3 = i32::from(self.gas.gas_3);
        let heat_range = i32::from(self.gas.heat_range);
        let heat_value = i32::from(self.gas.heat_value);

        let var_1 = (ambient_temp * gas_3 / 10) << 8;
        // division by 10 must not be last to avoid overflow
        let var_2 = (gas_1 + 784) / 10 * (((gas_2 + 154009) * target_temp * 5 / 100) + 3276800);
        let var_3 = var_1 + (var_2 >> 1);
        let var_4 = var_3 / (heat_range + 4);
        let var_5 = 131 * heat_value + 65536;
        let res_heat_x100 = (var_4 / var_5 - 250) * 34;
        ((res_heat_x100 + 50) / 100).try_into().unwrap()
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

    fn read_gas_params(device: &I2c) -> BmeResult<GasParams> {
        let mut buffer = [0; 4];
        device
            .i2c_read(REG_PAR_G, &mut buffer)
            .map_err(BmeError::ParamError)?;
        let buf_checked = &buffer[..4];

        // par_g1 bits, 0xED
        let par = buf_checked[2..3].try_into().unwrap();
        let gas_1 = i8::from_le_bytes(par);
        // par_g2 bits, 0xEC, 0xEB
        let par = buf_checked[0..2].try_into().unwrap();
        let gas_2 = i16::from_le_bytes(par);
        // par_g3 bits, 0xEE
        let par = buf_checked[3..4].try_into().unwrap();
        let gas_3 = i8::from_le_bytes(par);

        device
            .i2c_read(REG_RES_HEAT_RANGE, &mut buffer[0..1])
            .map_err(BmeError::ParamError)?;
        let par = buffer[0..1].try_into().unwrap();
        let heat_range = (u8::from_le_bytes(par) & 0b00110000) >> 4;
        device
            .i2c_read(REG_RES_HEAT_VAL, &mut buffer[0..1])
            .map_err(BmeError::ParamError)?;
        let par = buffer[0..1].try_into().unwrap();
        let heat_value = i8::from_le_bytes(par);
        device
            .i2c_read(REG_RANGE_SWITCHING_ERROR, &mut buffer[0..1])
            .map_err(BmeError::ParamError)?;
        let par = buffer[0..1].try_into().unwrap();
        let range_switching_error = i8::from_le_bytes(par) >> 4;

        Ok(GasParams {
            gas_1,
            gas_2,
            gas_3,
            heat_range,
            heat_value,
            range_switching_error,
        })
    }

    // calculation as definted in BME680 datasheet (page 17)
    // t_fine parameter returned for use in pressure calculation (see page 18)
    fn calculate_temperature(&self, raw_temp: u32) -> (f64, f64) {
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
    fn calculate_pressure(&self, raw_pres: u32, t_fine: f64) -> f64 {
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
    fn calculate_humidity(&self, raw_hum: u16, temp: f64) -> f64 {
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

    fn calculate_gas(&self, gas: (u16, bool, u8)) -> BmeResult<f64> {
        let (gas_val, heater_stability, heat_range) = gas;
        let gas_val = f64::from(gas_val);
        let heat_range = usize::from(heat_range);
        let params = &self.gas;
        let range_switching_error = f64::from(params.range_switching_error);

        let var_1 = (1340.0 + 5.0 * range_switching_error) * GAS_ARRAY_1[heat_range];
        let gas = var_1 * GAS_ARRAY_2[heat_range] / (gas_val - 512.0 + var_1);

        match heater_stability {
            true => Ok(gas),
            false => Err(BmeError::HeatError(gas)),
        }
    }
}

#[derive(Debug, Error)]
pub enum BmeError {
    #[error("failed to open device at address 0x{:X}", I2C_ADDRESS)]
    OpenError(#[source] I2cError),
    #[error("failed to set force mode")]
    ForceError(#[source] I2cError),
    #[error("failed to read measurements")]
    MeasurementError(#[source] I2cError),
    #[error("failed to read measurement status")]
    StatusError(#[source] I2cError),
    #[error("failed to read calibration parameters")]
    ParamError(#[source] I2cError),
    #[error("failed to apply parameters")]
    SetError(#[source] I2cError),
    #[error("failed to reset")]
    ResetError(#[from] ResetError),
    #[error("{0} is an invalid heater time. Valid values range from 0 to 63")]
    TimeError(u8),
    #[error("target heater temperature not reached. target temperature may be too high, or wait time may be too low")]
    HeatError(f64),
}

#[derive(Debug, Error)]
pub enum ResetError {
    #[error(transparent)]
    I2cError(I2cError),
    #[error("device values not reset")]
    ResetFail,
}
