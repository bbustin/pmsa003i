use crate::error::Error;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;

#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

use crate::types::Reading;
pub(crate) const RESPONSE_LENGTH: usize = 32;

/// Driver for the PMSA003I sensor
///
/// blocking example:
/// ```
///    info!("Setting up PMSA003I sensor");
///    let mut pmsa003i = Pmsa003i::new(i2c);
///
///    info!("Reading PMSA003I sensor");
///    match pmsa003i.read() {
///        Ok(result) => info!("PMSA003I reading: {reading:?}"),
///        Err(error) => error!("Unable to read PMSA003I sensor. Error: {error:?}"),
///    };
///```
///
/// async example:
/// ```
///    info!("Setting up PMSA003I sensor");
///    let mut pmsa003i = Pmsa003i::new(i2c);
///
///    info!("Reading PMSA003I sensor");
///    match pmsa003i.read().await {
///        Ok(result) => info!("PMSA003I reading: {reading:?}"),
///        Err(error) => error!("Unable to read PMSA003I sensor. Error: {error:?}"),
///    };
///```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Pmsa003i<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> Pmsa003i<I2C> {
    /// Creates a new driver instance using the given I2C bus and address 0x12
    pub fn new(i2c: I2C) -> Self {
        Pmsa003i { i2c, address: 0x12 }
    }

    /// Destroys the driver and returns the used I2C bus
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Blocking read
    #[cfg(any(not(feature = "async"), doc))]
    pub fn read(&mut self) -> Result<Reading, Error<I2C::Error>> {
        let result = self.read_raw()?;
        Ok(Reading::from(result))
    }

    /// Async read
    #[cfg(any(feature = "async", doc))]
    pub async fn read(&mut self) -> Result<Reading, Error<I2C::Error>> {
        let result = self.read_raw().await?;
        Ok(Reading::from(result))
    }

    /// Blocking raw read
    #[cfg(not(feature = "async"))]
    pub fn read_raw(&mut self) -> Result<[u8; RESPONSE_LENGTH], Error<I2C::Error>> {
        let mut response = [0; RESPONSE_LENGTH];

        self.i2c
            .read(self.address, &mut response)
            .map_err(Error::I2C)?;

        if !self.is_start_of_frame(response) {
            return Err(Error::BadMagic);
        }

        self.check_data_integrity(response)?;

        Ok(response)
    }

    /// Async raw read
    #[cfg(feature = "async")]
    pub async fn read_raw(&mut self) -> Result<[u8; RESPONSE_LENGTH], Error<I2C::Error>> {
        let mut response = [0; RESPONSE_LENGTH];

        self.i2c
            .read(self.address, &mut response)
            .await
            .map_err(Error::I2C)?;

        if !self.is_start_of_frame(response) {
            return Err(Error::BadMagic);
        }

        self.check_data_integrity(response)?;

        Ok(response)
    }

    fn is_start_of_frame(&self, data: [u8; RESPONSE_LENGTH]) -> bool {
        data.get(0).unwrap() == &0x42 && data.get(1).unwrap() == &0x4d
    }

    fn check_data_integrity(&self, data: [u8; RESPONSE_LENGTH]) -> Result<(), Error<I2C::Error>> {
        let sum = data[0..RESPONSE_LENGTH - 2]
            .iter()
            .fold(0u16, |accum, next| accum + *next as u16);

        let expected_sum =
            u16::from_be_bytes([data[RESPONSE_LENGTH - 2], data[RESPONSE_LENGTH - 1]]);

        if sum == expected_sum {
            return Ok(());
        }

        Err(Error::BadChecksum)
    }
}
