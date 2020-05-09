

use crate::interface::SensorInterface;
use embedded_hal::blocking::i2c::Write;
use crate::Error;

/// This device supports multiple addresses depending on
/// the configuration of CAD0 and CAD1
/// The format of these address is ADDR_CAD0_CAD1_nBIT,
/// Where 0 indicates tie to ground, 1 to Vdd
/// If CAD0 and CAD1 are floating, I2C address will be 0x0E / 0x1C.
pub const ADDR_0_0_7BIT:u8 = 0x0C;
pub const ADDR_0_1_7BIT:u8 = 0x0D;
pub const ADDR_1_0_7BIT:u8 = 0x0E;
pub const ADDR_1_1_7BIT:u8 = 0x0F;
pub const ADDR_7BIT_DEFAULT:u8 = 0x0E;

pub const ADDR_0_0_8BIT:u8 = 0x18;
pub const ADDR_0_1_8BIT:u8 = 0x1A;
pub const ADDR_1_0_8BIT:u8 = 0x1C;
pub const ADDR_1_1_8BIT:u8 = 0x1E;
pub const ADDR_8BIT_DEFAULT:u8 = 0x1C;

/// The default 7-bit i2c address when CAD0 and CAD1 are left floating
pub const DEFAULT_ADDRESS:u8 = ADDR_7BIT_DEFAULT;


pub struct I2cInterface<I2C> {
    i2c_port: I2C,
    address: u8,
}

impl I2cInterface<I2C> {

    pub fn default(i2c_port: I2C) -> Self {
        Self::new(i2c_port, DEFAULT_ADDRESS)
    }

    pub fn new(i2c_port: I2C, address: u8) -> Self {
        Self {
            i2c_port,
            address,
        };
        //TODO wait 50 ms for sensor POR? or assume at least that much has elapsed since POR
        inst.reset()?;
        Ok(inst)
    }
}

impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>
        + hal::blocking::i2c::Read<Error = CommE>
        + hal::blocking::i2c::WriteRead<Error = CommE>,
{
    type InterfaceError = crate::Error<CommE>;

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        let write_buf = [reg, val];
        self.i2c_port
            .write(self.address, &write_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }


    fn read_block(&mut self, reg: u8, recv_buf: &mut [u8]) -> Result<(), Self::InterfaceError> {
        let cmd_buf = [reg];
        self.i2c_port
            .write_read(self.address, &cmd_buf, &mut recv_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }

}