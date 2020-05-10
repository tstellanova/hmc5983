use crate::interface::SensorInterface;
use crate::Error;
use embedded_hal as hal;


const I2C_READ_ADDRESS: u8 = 0x3D;
const I2C_WRITE_ADDRESS: u8 = 0x3C;

pub struct I2cInterface<I2C> {
    i2c_port: I2C,
}

impl<I2C> I2cInterface<I2C> {
    pub fn new(i2c_port: I2C, address: u8) -> Self {
        Self { i2c_port }
    }
}

impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
where
    I2C: hal::blocking::i2c::Write<Error = CommE>
        + hal::blocking::i2c::Read<Error = CommE>
        + hal::blocking::i2c::WriteRead<Error = CommE>,
    CommE: core::fmt::Debug,
{
    type InterfaceError = crate::Error<CommE, ()>;

    fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), Self::InterfaceError> {
        let write_buf = [reg, val];
        self.i2c_port
            .write(I2C_WRITE_ADDRESS, &write_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }

    fn read_block(
        &mut self,
        reg: u8,
        recv_buf: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        let cmd_buf = [reg];
        self.i2c_port
            .write_read(I2C_READ_ADDRESS, &cmd_buf, recv_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }
}
