# hmc5983

A rust embedded-hal driver for the 
Honeywell HMC5983 
and similar 3-axis magnetometers
such as the 
HMC5883 and the
Isentek IST8310. 

Note that some sensors will support both i2c and SPI
interfaces, while others (such as the IST8310) only
support a single interface.

## Status

- [x] Basic i2c setup support
- [ ] Basic spi setup support
- [x] read of main xyz magentometer vector
- [ ] Tests with mock embedded hal
- [ ] Periodic configuration check (for poor i2c connections)
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI
- [ ] support for cross-axis flow calibration





