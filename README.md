# hmc5983

A rust embedded-hal driver for the 
Honeywell HMC5983 
and similar 3-axis magnetometers
such as the 
HMC5883. 

Note that some devices (HMC5983) support both I2C and SPI
interfaces, while others (eg HMC5883, QMC5883) only
support a single interface (I2C).

# Example

You can connect to the HMC5983 through either I2C or SPI:

```
  use hmc5983::HMC5983;

  let mut mag_int = HMC5983::new_with_interface(
        hmc5983::interface::SpiInterface::new(spi_bus1.acquire(), spi_cs_mag),
    );
    mag_int.init(&mut delay_source).expect("mag_int init failed");


    let mut mag_ext = HMC5983::new_with_interface(
        hmc5983::interface::I2cInterface::new(i2c_bus1.acquire()) );
    mag_ext.init(&mut delay_source).expect("mag_ext init failed");
```

## Status

- [x] Basic i2c setup support
- [x] Basic spi setup support
- [x] read of main xyz magnetometer vector
- [ ] support for DRDY pin
- [ ] Tests with mock embedded hal
- [ ] Periodic configuration check (for poor i2c connections)
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI





