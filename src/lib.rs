/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use crate::interface::SensorInterface;

pub mod interface;


/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE> {
    /// Sensor communication error
    Comm(CommE),

    /// Sensor reading out of range
    OutOfRange,

    /// Configuration reads invalid
    Configuration,

    /// Unrecognized chip ID
    UnknownChipId,
}


/// Who Am I (WAI) register
pub const REG_WAI:u8 = 	0x00;
// Info register
// const REG_INFO:u8 = 0x01;
/// X-axis output value register
const REG_DATA_X:u8 = 0x03;
// Y-axis output value register
// const REG_DATA_Y:u8 = 0x05;
// Z-axis output value register
// const REG_DATA_Z:u8	= 0x07;

/// Register to read out all three dimensions of mag data
const REG_MAG_DATA_START:u8 = REG_DATA_X;

/// Control setting register 1
const REG_CTRL1: u8 =  0x0A;
/// Control setting register 2
pub const REG_CTRL2: u8 =  0x0B;

/// Averaging control register
pub const REG_AVG_CTRL:u8 = 0x41;

/// Sensor Selection register (mode select)
pub const REG_SENS_MODE_SELECT:u8 =  0x42;

// Status Register 1
// const REG_STATUS1: u8 = 0x02;
// Status Register 2
// const REG_STATUS2: u8 = 0x09;

/// Average 16 times
const AVG_CTRL_16X: u8 = 0x24;

/// Set Reset Pulse Duration: Low power mode
const SRPD_MODE_LOW_POWER: u8 = 0xC0;

const BLOCK_BUF_LEN: usize = 32;



pub struct HMC5893<SI> {
    pub(crate) sensor_interface: SI,
    /// Buffer for reads and writes to the sensor
    block_buf: [u8; BLOCK_BUF_LEN],
    /// Stores the requested averaging control configuration
    avg_ctrl_reg_set: u8,
    /// Stores the requested SRPD control configuration
    srpd_ctrl_reg_set: u8,
}

impl<SI, SE> HMC5893<SI>
    where
        SI: SensorInterface<SensorError = SE>,
        SE: core::fmt::Debug,
{

    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            block_buf: [0; BLOCK_BUF_LEN],
            avg_ctrl_reg_set: 0,
            srpd_ctrl_reg_set: 0
        }
    }

    pub fn init(&mut self) -> Result<(), Error<CommE>> {
        self.reset()
    }

    fn reset(&mut self) -> Result<(), Error<CommE>> {
        const SRST_POR_FLAG: u8 = 0x01 << 0;
        //const DRDY_POLARITY_FLAG: u8 = 0x01 << 2;
        //const DRDY_ENABLE_FLAG: u8 = 0x01 << 3;
        const EXPECTED_PROD_ID:u8 = 0x10;
        // perform power-on-reset POR sequence
        self.sensor_interface.write_reg(REG_CTRL2, SRST_POR_FLAG)?;

        //configure averaging
        self.avg_ctrl_reg_set = AVG_CTRL_16X;
        self.sensor_interface.write_reg(REG_AVG_CTRL, self.avg_ctrl_reg_set)?;

        //configure SRPD
        self.srpd_ctrl_reg_set = SRPD_MODE_LOW_POWER;
        self.sensor_interface.write_reg(REG_SENS_MODE_SELECT, self.srpd_ctrl_reg_set)?;

        //compare product ID against known product ID
        let product_id = self.read_reg(REG_WAI)?;
        if product_id != EXPECTED_PROD_ID {
            return Err(Error::UnknownChipId)
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
    fn read_reg(&mut self, reg: u8 ) -> Result<u8, Error<CommE>> {
        self.sensor_interface.read_block(reg,&mut self.block_buf[..1])?;
        Ok(self.block_buf[0])
    }


    /// Verify that a magnetometer reading is within
    /// the expected range.
    /// From section "3.4 Magnetic Sensor Specifications"
    /// in datasheet
    fn reading_in_range(sample: &[i16; 3]) -> bool {
        /// Maximum Dynamic Range for X and Y axes (micro Teslas)
        const MDR_XY_AXES: i16 = 1600;
        /// Maximum Dynamic Range for Z axis (micro Teslas)
        const MDR_Z_AXIS: i16 = 2500;
        /// Resolution (micro Teslas per LSB)
        const RESO_PER_BIT: f32 = 0.3;
        const MAX_VAL_XY: i16 = (((MDR_XY_AXES as f32) / RESO_PER_BIT) as i16)  + 1;
        const MAX_VAL_Z: i16 = (((MDR_Z_AXIS as f32) / RESO_PER_BIT) as i16) + 1;

        sample[0].abs() <  MAX_VAL_XY &&
            sample[1].abs() < MAX_VAL_XY &&
            sample[2].abs() < MAX_VAL_Z
    }

    /// Combine high and low bytes of i16 mag value
    fn raw_reading_to_i16(buf: &[u8], idx: usize) -> i16 {
        let val: i16 = (buf[idx] as i16) | ((buf[idx+1] as i16) << 8) ;
        val
    }

    pub fn get_mag_vector(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<[i16; 3], Error<CommE>> {
        const SINGLE_MEASURE_MODE: u8 = 0x01;
        const XYZ_DATA_LEN: usize = 6;

        // Activate single measurement mode
        self.write_reg(REG_CTRL1, SINGLE_MEASURE_MODE)?;
        // Allow sensor time to collect & average data (6 ms min for 16x averaging)
        delay_source.delay_ms(6);

        //get the actual data from the sensor
        self.sensor_interface.read_block(REG_MAG_DATA_START, &mut self.block_buf[..XYZ_DATA_LEN])?;
        let sample_i16 = [
            Self::raw_reading_to_i16(&self.block_buf, 0),
            Self::raw_reading_to_i16(&self.block_buf, 2),
            Self::raw_reading_to_i16(&self.block_buf, 4)
            ];

        if !Self::reading_in_range(&sample_i16) {
            return Err(Error::OutOfRange)
        }

        //TODO do cross-axis flow calibration?
        Ok(sample_i16)
    }

}




