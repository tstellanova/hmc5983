use crate::interface::SensorInterface;

use embedded_hal as hal;
use hal::digital::v2::{OutputPin};

use crate::Error;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

const TRANSFER_BUF_LEN: usize = 32;

const DIRECTION_READ: u8 = 1<<7;
const DIRECTION_WRITE: u8 = 0;
const MULTI_ADDRESS_INCREMENT: u8 = 1<<6;



pub struct SpiInterface<SPI, CS> {
    spi: SPI,
    cs: CS,
    transfer_buf: [u8; TRANSFER_BUF_LEN],
}

impl<SPI, CS> SpiInterface<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs, transfer_buf: [0; TRANSFER_BUF_LEN] }
    }
}

impl<SPI, CS, CommE, PinE> SensorInterface for SpiInterface<SPI, CS>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CS: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    type InterfaceError = crate::Error<CommE, PinE>;

    fn read_block(
        &mut self,
        reg: u8,
        recv_buf: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        self.cs.set_low().map_err(Error::Pin)?;

        // the first byte in SPI receive is garbage
        let total_read_bytes = recv_buf.len() + 1;
        //bit 0: READ bit. The value is 1 on read, 0 on write.
        //bit 1: MS bit. When 0 don't increment address: when 1 increment address in multiple read.
        //bit 2-7: address AD(5:0). This is the address field of the indexed register.

        for i in 0..total_read_bytes {
            self.transfer_buf[i] = 0;
        }
        self.transfer_buf[0] = reg | DIRECTION_READ | MULTI_ADDRESS_INCREMENT;

        let rc =
            self.spi.transfer(self.transfer_buf[..total_read_bytes].as_mut()).map_err(Error::Comm);
        //release SPI bus
        self.cs.set_high().map_err(Error::Pin)?;

        if rc.is_err() {
            return Err(rc.unwrap_err());
        }

        let read_slice = rc.unwrap();
        recv_buf.copy_from_slice(&read_slice[1..total_read_bytes]);

        // #[cfg(feature = "rttdebug")]
        // rprintln!("read_slice: {:?}",read_slice);

        Ok(())
    }

    fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), Self::InterfaceError> {
        self.cs.set_low().map_err(Error::Pin)?;

        let block = [reg | DIRECTION_WRITE, val];
        let rc = self.spi.write(&block).map_err(Error::Comm);
        self.cs.set_high().map_err(Error::Pin)?;

        if rc.is_err() {
            return Err(rc.unwrap_err());
        }

        Ok(())
    }
}
