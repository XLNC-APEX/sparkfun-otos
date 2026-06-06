use embedded_hal_async::{digital::Wait, i2c::I2c};

use crate::{DEFAULT_ADDR, PRODUCT_ID, Result, error::Error, registers::Register};

pub struct SparkfunOTOS<I2C, IrqPin> {
    i2c: I2C,
    irq_pin: IrqPin,
}

impl<I2C, IrqPin> SparkfunOTOS<I2C, IrqPin>
where
    I2C: I2c,
    IrqPin: Wait,
{
    pub fn new(i2c: I2C, irq_pin: IrqPin) -> Self {
        Self { i2c, irq_pin }
    }

    pub async fn init(&mut self) -> Result<()> {
        self.check_product_id().await
    }

    async fn wait_for_data(&mut self) -> Result<()> {
        self.irq_pin
            .wait_for_low()
            .await
            .map_err(|_| Error::PinError)
    }

    pub async fn get_version(&mut self) -> Result<Version> {
        let rx = self.read_regs::<2>(Register::HW_VERSION).await?;
        Ok(Version {
            hw: rx[0],
            fw: rx[1],
        })
    }

    async fn check_product_id(&mut self) -> Result<()> {
        if self.read_reg(Register::PRODUCT_ID).await? == PRODUCT_ID {
            Ok(())
        } else {
            Err(Error::IncorrectProductID)
        }
    }

    async fn read_regs<const N: usize>(&mut self, reg: u8) -> Result<[u8; N]> {
        let mut rx = [0u8; N];
        self.i2c
            .write_read(DEFAULT_ADDR, &[reg], &mut rx)
            .await
            .map_err(|_| Error::I2cError)?;
        Ok(rx)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8> {
        let mut rx = [0];
        self.i2c
            .write_read(DEFAULT_ADDR, &[reg], &mut rx)
            .await
            .map_err(|_| Error::I2cError)?;
        Ok(rx[0])
    }

    // pub async fn get(&mut self) -> Result<()> {
    //     self.wait_for_data().await?;
    //     self.read().await
    // }
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    pub x: f32,
    pub y: f32,
    pub h: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Version {
    pub hw: u8,
    pub fw: u8,
}

// use embedded_hal::{
//     delay::DelayNs,
//     digital::{InputPin, OutputPin},
// };

// use crate::error::Error;

// pub struct SparkfunOTOS<PIN, D> {
//     pin: PIN,
//     delay: D,
// }

// const TIMEOUT_US: u8 = 100;

// impl<PIN, D, E> SparkfunOTOS<PIN, D>
// where
//     PIN: InputPin<Error = E> + OutputPin<Error = E>,
//     D: DelayNs,
// {
//     fn new(pin: PIN, delay: D) -> Self {
//         Self { pin, delay }
//     }

//     fn wait_for_state<F>(delay: &mut D, mut condition: F) -> Result<()>
//     where
//         F: FnMut() -> Result<bool, E>,
//     {
//         for _ in 0..TIMEOUT_US {
//             if condition()? {
//                 return Ok(());
//             }
//             delay.delay_us(1);
//         }
//         Err(OTOSError::Timeout)
//     }

//     fn wait_for_high(&mut self) -> Result<(), Error<E>> {
//         Self::wait_for_state(&mut self.delay, || self.pin.is_high())
//     }

//     fn wait_for_low(&mut self) -> Result<(), Error<E>> {
//         Self::wait_for_state(&mut self.delay, || self.pin.is_low())
//     }
// }
