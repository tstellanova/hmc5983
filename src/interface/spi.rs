use crate::interface::SensorInterface;

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use crate::Error;

pub struct SpiInterface<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl SpiInterface<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }
}

impl<SPI, CS, CommE, PinE> SensorInterface for SpiInterface<SPI, CS>
where
    SPI: hal::blocking::spi::Write<Error = CommE>
        + hal::blocking::spi::Transfer<Error = CommE>,
    CS: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    type InterfaceError = crate::Error<CommE>;

    fn read_block(
        &mut self,
        reg: u8,
        recv_buf: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        self.cs.set_low().map_err(Error::Pin)?;
        self.cs.set_high().map_err(Error::Pin)?;

        unimplemented!()

        // self.cs.set_high().map_err(Error::Pin)?;
        // if rc.is_err() {
        //     return Err(rc.unwrap_err());
        // }
        //
        // let max_len = if recv_buf
        //
        // Ok(())
    }

    fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), Self::InterfaceError> {
        if let Ok(_) = self.cs.set_low() {
            let block = [reg, val];
            let rc = self.spi.write(&block).map_err(Error::Comm);
            let _ = self.cs.set_high();

            if rc.is_err() {
                return Err(rc.unwrap_err());
            }
        }
        Ok(())
    }
}
