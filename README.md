# hmc5983

A rust embedded-hal driver for the 
Honeywell HMC5983 
and similar 3-axis magnetometers
such as the 
HMC5883. 

Note that some devices (HMC5983) support both I2C and SPI
interfaces, while others (eg HMC5883, QMC5883) only
support a single interface (I2C).

## Status

- [x] Basic i2c setup support
- [ ] Basic spi setup support
- [x] read of main xyz magnetometer vector
- [ ] Tests with mock embedded hal
- [ ] Periodic configuration check (for poor i2c connections)
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI





