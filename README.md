# HMC5883 async driver

A rust embedded-hal driver for the Honeywell HMC5883. Forked from the original work of [Todd Stellanova](https://github.com/tstellanova/hmc5983) and made async.

HMC5883 only support a single interface (I2C).

# Example

You can connect to the HMC5883 through I2C:

```rust
    use hmc5883_async::*;
    let mut hmc = HMC5883::new(i2c);

    hmc.init(&mut Delay).await.expect("init failed");

    loop {
        if let Ok(temp) = hmc.get_temperature().await {
            info!("Temperature: {:?}", temp);
        }
        match hmc.get_mag_vector().await {
            Ok(mag) => info!("Magnitude vector: {:?}", mag),
            Err(E) => info!("Printing Error {}", E),
        }

        Timer::after(Duration::from_secs(3)).await;
    }
```






