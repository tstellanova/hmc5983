/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]


#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

use crate::interface::SensorInterface;
use embedded_hal as hal;
use hal::blocking::delay::DelayMs;

pub mod interface;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// Sensor reading out of range
    OutOfRange,

    /// Configuration reads invalid
    Configuration,

    /// Unrecognized chip ID
    UnknownChipId,
}

/// Gain settings
/// gain in LSb/Gauss
/// One tesla (T) is equal to 104 gauss,
#[repr(u8)]
pub enum GainSetting {
    ///± 0.88 Ga  / 0.73 (mGa/LSb)
    Gain1370 = 0b00000000,
    ///± 1.30 Ga  / 0.92 (mGa/LSb)
    Gain1090 = 0b00100000,
    ///± 1.90 Ga  / 1.22 (mGa/LSb)
    Gain0820 = 0b01000000,
    ///± 2.50 Ga  / 1.52 (mGa/LSb)
    Gain0660 = 0b01100000,
    ///± 4.00 Ga  / 2.27 (mGa/LSb)
    Gain0440 = 0b10000000,
    ///± 4.70 Ga  / 2.56 (mGa/LSb)
    Gain0390 = 0b10100000,
    ///± 5.60 Ga  / 3.03 (mGa/LSb)
    Gain0330 = 0b11000000,
    ///± 8.10 Ga  / 4.35 (mGa/LSb)
    Gain0230 = 0b11100000,
}

#[repr(u8)]
pub enum OdrSetting {
    Odr_0_75_Hz = 0b000,
    Odr_1_5_Hz = 0b001,
    Odr_3_0_Hz = 0b010,
    Odr_7_5_Hz = 0b011,
    Odr_15_0_Hz = 0b100,
    Odr_30_0_Hz = 0b110,
    Odr_220_0_Hz = 0b111,
}

#[repr(u8)]
pub enum SampleAvgSetting {
    AvgSamples1 = 0b00,
    AvgSamples2 = 0b01,
    AvgSamples4 = 0b10,
    AvgSamples8 = 0b11,
}

#[repr(u8)]
pub enum MeasurementModeSetting {
    NormalMode = 0b00,
    PositiveBias = 0b01,
    NegativeBias = 0b10,
    TemperatureOnly = 0b11,
}

const REG_CONFIG_A: u8 = 0x00;
const REG_CONFIG_B: u8 = 0x01;
const REG_CONFIG_C: u8 = 0x02;

/// X-axis output value register
const REG_DATA_X: u8 = 0x03;
// Y-axis output value register
// const REG_DATA_Y:u8 = 0x05;
// Z-axis output value register
// const REG_DATA_Z:u8	= 0x07;

// const REG_STATUS:u8 = 0x09;

/// Register to read out all three dimensions of mag data
const REG_MAG_DATA_START: u8 = REG_DATA_X;

/// Identification Register A
const REG_ID_A: u8 = 0x0A;
/// Identification Register B
const REG_ID_B: u8 = 0x0B;
/// Identification Register C
const REG_ID_C: u8 = 0x0C;

// Temperature outputs, HMC5983
// const REG_TEMP_OUTPUT_MSB: u8 = 0x31;
// const REG_TEMP_OUTPUT_LSB: u8 = 0x32;

// Status Register 2
// const REG_STATUS2: u8 = 0x09;

const BLOCK_BUF_LEN: usize = 32;

pub struct HMC5983<SI> {
    pub(crate) sensor_interface: SI,
    /// Buffer for reads and writes to the sensor
    block_buf: [u8; BLOCK_BUF_LEN],


}

impl<SI, CommE, PinE> HMC5983<SI>
where
    SI: SensorInterface<InterfaceError = crate::Error<CommE, PinE>>,
{
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            block_buf: [0; BLOCK_BUF_LEN],
        }
    }

    pub fn init(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), crate::Error<CommE, PinE>> {
        self.reset(delay_source)
    }

    fn reset(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), crate::Error<CommE, PinE>> {

        #[cfg(feature = "rttdebug")]
        for reg in 0x00..0x0D {
            let val = self.read_reg(reg)?;
            rprintln!("0x{:0x} : {} ", reg, val);
        }

        const EXPECTED_PROD_ID_A: u8 = 72; //'H';
        const EXPECTED_PROD_ID_B: u8 = 52; //'4';
        const EXPECTED_PROD_ID_C: u8 = 51; //'3';
        //compare product ID against known product ID
        //read the product identifiers
        self.sensor_interface.read_block(REG_ID_A,&mut self.block_buf[..3])?;
        if self.block_buf[0] != EXPECTED_PROD_ID_A ||
            self.block_buf[1] != EXPECTED_PROD_ID_B ||
            self.block_buf[2] != EXPECTED_PROD_ID_C {

            #[cfg(feature = "rttdebug")]
            rprintln!("bad ID block: {},{},{}",
                self.block_buf[0], self.block_buf[1], self.block_buf[2]);

            return Err(Error::UnknownChipId);
        }

        // Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default, normal measurement)
        // REG_CONFIG_A can set the Output Data Rate (ODR)
        //

        self.set_all_config_a(MeasurementModeSetting::NormalMode,
        OdrSetting::Odr_30_0_Hz,
            SampleAvgSetting::AvgSamples8,
            true
        )?;

        self.set_gain(GainSetting::Gain0820)?;
        // Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
        self.sensor_interface.write_reg(REG_CONFIG_C, 0x00)?;
        delay_source.delay_ms(100);

        Ok(())
    }

    pub fn set_gain(&mut self, gain: GainSetting) -> Result<(), crate::Error<CommE, PinE>>  {
        let gain_val: u8 = gain as u8;
        self.sensor_interface.write_reg(REG_CONFIG_B, gain_val)?;

        let confirm_val = self.read_reg(REG_CONFIG_B)?;
        if confirm_val != gain_val {
            #[cfg(feature = "rttdebug")]
            rprintln!("gain bad: expected {} got {}",gain_val, confirm_val);
            return Err(Error::Configuration);
        }
        Ok(())
    }

    pub fn set_all_config_a(&mut self,
        mode: MeasurementModeSetting,
        odr: OdrSetting,
        averaging: SampleAvgSetting,
        temp_enabled: bool) -> Result<(), crate::Error<CommE, PinE>> {

        let new_val =
            (if temp_enabled { (1 << 7) } else {0}) &
                ((averaging as u8) << 6) &
                ((odr as u8) << 4) &
                ((mode as u8) << 2);
        self.sensor_interface.write_reg(REG_CONFIG_A, new_val)
    }



    /// Read a single register
    fn read_reg(&mut self, reg: u8) -> Result<u8, crate::Error<CommE, PinE>> {
        self.sensor_interface
            .read_block(reg, &mut self.block_buf[..1])?;
        Ok(self.block_buf[0])
    }

    /// Verify that a magnetometer reading is within the expected range.
    // fn reading_in_range(sample: &[i16; 3]) -> bool {
    //     /// Maximum Dynamic Range for X and Y axes (micro Teslas)
    //     const MDR_XY_AXES: i16 = 1600;
    //     /// Maximum Dynamic Range for Z axis (micro Teslas)
    //     const MDR_Z_AXIS: i16 = 2500;
    //     /// Resolution (micro Teslas per LSB)
    //     const RESO_PER_BIT: f32 = 0.3;
    //     const MAX_VAL_XY: i16 =
    //         (((MDR_XY_AXES as f32) / RESO_PER_BIT) as i16) + 1;
    //     const MAX_VAL_Z: i16 =
    //         (((MDR_Z_AXIS as f32) / RESO_PER_BIT) as i16) + 1;
    //
    //     sample[0].abs() < MAX_VAL_XY
    //         && sample[1].abs() < MAX_VAL_XY
    //         && sample[2].abs() < MAX_VAL_Z
    // }

    /// Combine high and low bytes of i16 mag value
    fn raw_reading_to_i16(buf: &[u8], idx: usize) -> i16 {
        let val: i16 = (buf[idx] as i16) | ((buf[idx + 1] as i16) << 8);
        val
    }

    pub fn get_mag_vector(
        &mut self,
    ) -> Result<[i16; 3], crate::Error<CommE, PinE>> {
        const XYZ_DATA_LEN: usize = 6;

        //get the actual data from the sensor
        self.sensor_interface.read_block(
            REG_MAG_DATA_START,
            &mut self.block_buf[..XYZ_DATA_LEN],
        )?;
        let sample_i16 = [
            Self::raw_reading_to_i16(&self.block_buf, 0),
            Self::raw_reading_to_i16(&self.block_buf, 2),
            Self::raw_reading_to_i16(&self.block_buf, 4),
        ];

        // if !Self::reading_in_range(&sample_i16) {
        //     #[cfg(feature = "rttdebug")]
        //     rprintln!("bad reading?");
        //
        //     return Err(Error::OutOfRange);
        // }

        //TODO do cross-axis flow calibration?
        Ok(sample_i16)
    }
}
