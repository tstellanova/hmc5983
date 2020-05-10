pub mod spi;
pub use self::spi::SpiInterface;

pub mod i2c;
pub use self::i2c::I2cInterface;

/// A method of communicating with the device
pub trait SensorInterface {
    /// Interface associated error type
    type InterfaceError;

    /// Read a block from a specific register
    /// `reg`: The register address to read from
    /// `recv_buf`: The buffer to receive into
    fn read_block(
        &mut self,
        reg: u8,
        recv_buf: &mut [u8],
    ) -> Result<(), Self::InterfaceError>;

    /// Write a value to a register
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError>;
}
