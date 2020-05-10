/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

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

const REG_CONFIG_A: u8 = 0x00;
const REG_CONFIG_B: u8 = 0x01;
const REG_CONFIG_C: u8 = 0x02;

/// X-axis output value register
const REG_DATA_X: u8 = 0x03;
// Y-axis output value register
// const REG_DATA_Y:u8 = 0x05;
// Z-axis output value register
// const REG_DATA_Z:u8	= 0x07;

const REG_STATUS:u8 = 0x09;

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


// Status Register 1
// const REG_STATUS1: u8 = 0x02;
// Status Register 2
// const REG_STATUS2: u8 = 0x09;

/// Average 16 times
const AVG_CTRL_16X: u8 = 0x24;

/// Set Reset Pulse Duration: Low power mode
const SRPD_MODE_LOW_POWER: u8 = 0xC0;

const BLOCK_BUF_LEN: usize = 32;

pub struct HMC5983<SI> {
    pub(crate) sensor_interface: SI,
    /// Buffer for reads and writes to the sensor
    block_buf: [u8; BLOCK_BUF_LEN],
    /// Stores the requested averaging control configuration
    avg_ctrl_reg_set: u8,
    /// Stores the requested SRPD control configuration
    srpd_ctrl_reg_set: u8,
}

impl<SI, CommE, PinE> HMC5983<SI>
where
    SI: SensorInterface<InterfaceError = crate::Error<CommE, PinE>>,
{
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            block_buf: [0; BLOCK_BUF_LEN],
            avg_ctrl_reg_set: 0,
            srpd_ctrl_reg_set: 0,
        }
    }

    pub fn init(&mut self, delay_source: &mut DelayMs<u8>) -> Result<(), crate::Error<CommE, PinE>> {
        self.reset(delay_source)
    }

    fn reset(&mut self, delay_source: &mut DelayMs<u8>) -> Result<(), crate::Error<CommE, PinE>> {
        const EXPECTED_PROD_ID_A: u8 = 72; //'H';
        const EXPECTED_PROD_ID_B: u8 = 52; //'4';
        const EXPECTED_PROD_ID_C: u8 = 51; //'3';

        // Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default, normal measurement)
        self.sensor_interface.write_reg(REG_CONFIG_A, 0x70)?;
        // Set gain: Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5)
        self.sensor_interface.write_reg(REG_CONFIG_B, 0xA0)?;
        // Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
        self.sensor_interface.write_reg(REG_CONFIG_C, 0x00)?;
        delay_source.delay_ms(70);


        //compare product ID against known product ID
        //read three sequential ID bytes
        self.sensor_interface.read_block(REG_ID_A,&mut self.block_buf[..3])?;
        if self.block_buf[0] != EXPECTED_PROD_ID_A ||
            self.block_buf[1] != EXPECTED_PROD_ID_B ||
            self.block_buf[2] != EXPECTED_PROD_ID_C {
            return Err(Error::UnknownChipId);
        }


        Ok(())
    }

    ///	 Write a block to a specific register
    /// reg: The register address to write to
    /// send_buf: The buffer to send
    // fn write_block(&mut self, reg: u8, send_buf: &[u8]) -> Result<(), Error<CommE>>{
    //     self.block_buf[0] = reg;
    //     //this panics if send_buf is bigger than expected:
    //     assert!(send_buf.len() <= self.block_buf.len() + 1);
    //     self.block_buf[1..send_buf.len()+1].copy_from_slice(send_buf);
    //     self.i2c_port
    //         .write(self.address, &self.block_buf)
    //         .map_err(Error::Comm)?;
    //     Ok(())
    // }

    /// Read a single register
    fn read_reg(&mut self, reg: u8) -> Result<u8, crate::Error<CommE, PinE>> {
        self.sensor_interface
            .read_block(reg, &mut self.block_buf[..1])?;
        Ok(self.block_buf[0])
    }

    /// Verify that a magnetometer reading is within the expected range.
    /// From section "3.4 Magnetic Sensor Specifications" in datasheet
    fn reading_in_range(sample: &[i16; 3]) -> bool {
        /// Maximum Dynamic Range for X and Y axes (micro Teslas)
        const MDR_XY_AXES: i16 = 1600;
        /// Maximum Dynamic Range for Z axis (micro Teslas)
        const MDR_Z_AXIS: i16 = 2500;
        /// Resolution (micro Teslas per LSB)
        const RESO_PER_BIT: f32 = 0.3;
        const MAX_VAL_XY: i16 =
            (((MDR_XY_AXES as f32) / RESO_PER_BIT) as i16) + 1;
        const MAX_VAL_Z: i16 =
            (((MDR_Z_AXIS as f32) / RESO_PER_BIT) as i16) + 1;

        sample[0].abs() < MAX_VAL_XY
            && sample[1].abs() < MAX_VAL_XY
            && sample[2].abs() < MAX_VAL_Z
    }

    /// Combine high and low bytes of i16 mag value
    fn raw_reading_to_i16(buf: &[u8], idx: usize) -> i16 {
        let val: i16 = (buf[idx] as i16) | ((buf[idx + 1] as i16) << 8);
        val
    }

    pub fn get_mag_vector(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<[i16; 3], crate::Error<CommE, PinE>> {
        const SINGLE_MEASURE_MODE: u8 = 0x01;
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

        if !Self::reading_in_range(&sample_i16) {
            return Err(Error::OutOfRange);
        }

        //TODO do cross-axis flow calibration?
        Ok(sample_i16)
    }
}
