#![no_std]

//! # SparkFun OTOS driver
//!
//! A driver for SparkFun Optical Tracking Odometry Sensor using I2C

use embedded_hal::i2c::I2c;

const ADDR: u8 = 0x17;

#[derive(Debug)]
pub enum Error<E> {
    Communication(E),
}

#[allow(dead_code)]
struct Register;

#[allow(dead_code)]
impl Register {
    const OFFSETS: u8 = 0x10;

    const POS: u8 = 0x20;

    const POS_X_L: u8 = 0x20;
    const POS_X_H: u8 = 0x21;
    const POS_Y_L: u8 = 0x22;
    const POS_Y_H: u8 = 0x23;
    const POS_HEADING_L: u8 = 0x24;
    const POS_HEADING_H: u8 = 0x25;

    const VEL: u8 = 0x26;

    const VEL_X_L: u8 = 0x26;
    const VEL_X_H: u8 = 0x27;
    const VEL_Y_L: u8 = 0x28;
    const VEL_Y_H: u8 = 0x29;
    const VEL_HEADING_L: u8 = 0x2A;
    const VEL_HEADING_H: u8 = 0x2B;

    const ACCEL: u8 = 0x2C;

    const ACCEL_X_L: u8 = 0x2C;
    const ACCEL_X_H: u8 = 0x2D;
    const ACCEL_Y_L: u8 = 0x2E;
    const ACCEL_Y_H: u8 = 0x2F;
    const ACCEL_HEADING_L: u8 = 0x30;
    const ACCEL_HEADING_H: u8 = 0x31;

    const POS_SD: u8 = 0x32;

    const POS_X_L_SD: u8 = 0x32;
    const POS_X_H_SD: u8 = 0x33;
    const POS_Y_L_SD: u8 = 0x34;
    const POS_Y_H_SD: u8 = 0x35;
    const POS_HEADING_L_SD: u8 = 0x36;
    const POS_HEADING_H_SD: u8 = 0x37;

    const VEL_SD: u8 = 0x38;

    const VEL_X_L_SD: u8 = 0x38;
    const VEL_X_H_SD: u8 = 0x39;
    const VEL_Y_L_SD: u8 = 0x3A;
    const VEL_Y_H_SD: u8 = 0x3B;
    const VEL_HEADING_L_SD: u8 = 0x3C;
    const VEL_HEADING_H_SD: u8 = 0x3D;

    const ACCEL_SD: u8 = 0x3E;

    const ACCEL_X_L_SD: u8 = 0x3E;
    const ACCEL_X_H_SD: u8 = 0x3F;
    const ACCEL_Y_L_SD: u8 = 0x40;
    const ACCEL_Y_H_SD: u8 = 0x41;
    const ACCEL_HEADING_L_SD: u8 = 0x42;
    const ACCEL_HEADING_H_SD: u8 = 0x43;
}

pub struct SparkFunOTOS<I2C> {
    i2c: I2C,
}

impl<I2C> SparkFunOTOS<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    fn read_triple(&mut self, addr: u8) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut rx_buf = [0u8; 6];
        match self.i2c.write_read(addr, &[Register::POS], &mut rx_buf) {
            Ok(()) => Ok((
                i16::from_le_bytes([rx_buf[0], rx_buf[1]]),
                i16::from_le_bytes([rx_buf[2], rx_buf[3]]),
                i16::from_le_bytes([rx_buf[4], rx_buf[5]]),
            )),
            Err(e) => Err(Error::Communication(e)),
        }
    }

    pub fn reset_pos(&mut self) -> Result<(), I2C::Error> {
        let tx = [Register::POS, 0, 0, 0, 0, 0, 0];
        self.i2c.write(ADDR, &tx)?;
        Ok(())
    }

    pub fn get_pose(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::POS)
    }

    pub fn get_velocity(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::VEL)
    }

    pub fn get_accelleration(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::ACCEL)
    }

    pub fn get_pose_sd(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::POS_SD)
    }

    pub fn get_velocity_sd(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::VEL_SD)
    }

    pub fn get_accelleration_sd(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        self.read_triple(Register::ACCEL_SD)
    }

    pub fn set_offsets(&mut self, x: i16, y: i16, h: i16) -> Result<(), I2C::Error> {
        let lx = x.to_le_bytes();
        let ly = y.to_le_bytes();
        let lh = h.to_le_bytes();
        let tx = [Register::OFFSETS, lx[0], lx[1], ly[0], ly[1], lh[0], lh[1]];
        self.i2c.write(ADDR, &tx)?;
        Ok(())
    }
}
