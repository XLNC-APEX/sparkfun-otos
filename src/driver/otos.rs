use arrayref::array_ref;
use bitfield_struct::bitfield;
use embedded_hal_async::{
    digital::Wait,
    i2c::{I2c, Operation},
};

use crate::{
    DEFAULT_ADDR, K_I16_TO_METER, K_I16_TO_MPS, K_I16_TO_MPSS, K_I16_TO_RAD, K_I16_TO_RPS,
    K_I16_TO_RPSS, K_METER_TO_I16, K_MPS_TO_I16, K_MPSS_TO_I16, K_RAD_TO_I16, K_RPS_TO_I16,
    K_RPSS_TO_I16, MAX_SCALAR, MIN_SCALAR, PRODUCT_ID, Result, error::Error, registers::Register,
};

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

    pub async fn get_pos(&mut self) -> Result<Pose> {
        self.read_pose(Register::POS, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }
    pub async fn get_vel(&mut self) -> Result<Pose> {
        self.read_pose(Register::VEL, K_I16_TO_MPS, K_I16_TO_RPS)
            .await
    }
    pub async fn get_acc(&mut self) -> Result<Pose> {
        self.read_pose(Register::ACCEL, K_I16_TO_MPSS, K_I16_TO_RPSS)
            .await
    }
    pub async fn get_pos_vel_acc(&mut self) -> Result<[Pose; 3]> {
        self.read_poses(Register::POS).await
    }
    // SD versions
    pub async fn get_pos_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::POS_SD, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }
    pub async fn get_vel_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::VEL_SD, K_I16_TO_MPS, K_I16_TO_RPS)
            .await
    }
    pub async fn get_acc_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::ACCEL_SD, K_I16_TO_MPSS, K_I16_TO_RPSS)
            .await
    }
    pub async fn get_pos_vel_acc_sd(&mut self) -> Result<[Pose; 3]> {
        self.read_poses(Register::POS_SD).await
    }

    async fn read_pose(&mut self, reg: u8, k_xy: f32, k_h: f32) -> Result<Pose> {
        self.wait_for_data().await?;
        let rx = self.read_regs::<6>(reg).await?;
        Ok(Pose::parse(&rx, k_xy, k_h))
    }

    async fn read_poses(&mut self, reg: u8) -> Result<[Pose; 3]> {
        self.wait_for_data().await?;
        let rx = self.read_regs::<18>(reg).await?;
        Ok([
            Pose::parse(array_ref![rx, 0, 6], K_I16_TO_METER, K_I16_TO_RAD),
            Pose::parse(array_ref![rx, 6, 6], K_I16_TO_METER, K_I16_TO_RAD),
            Pose::parse(array_ref![rx, 12, 6], K_I16_TO_METER, K_I16_TO_RAD),
        ])
    }

    pub async fn set_pos(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::POS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }

    pub async fn set_vel(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::VEL, K_MPS_TO_I16, K_RPS_TO_I16)
            .await
    }

    pub async fn set_acc(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::ACCEL, K_MPSS_TO_I16, K_RPSS_TO_I16)
            .await
    }

    pub async fn set_pos_vel_acc(&mut self, poses: &[Pose; 3]) -> Result<()> {
        self.write_poses(poses, Register::POS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }

    pub async fn set_pos_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::POS_SD, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }

    pub async fn set_vel_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::VEL_SD, K_MPS_TO_I16, K_RPS_TO_I16)
            .await
    }

    pub async fn set_acc_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::ACCEL_SD, K_MPSS_TO_I16, K_RPSS_TO_I16)
            .await
    }

    pub async fn set_pos_vel_acc_sd(&mut self, poses: &[Pose; 3]) -> Result<()> {
        self.write_poses(poses, Register::POS_SD, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }

    async fn write_pose(&mut self, pose: &Pose, reg: u8, k_xy: f32, k_h: f32) -> Result<()> {
        self.write_regs(reg, &pose.encode(k_xy, k_h)).await
    }

    async fn write_poses(&mut self, poses: &[Pose; 3], reg: u8, k_xy: f32, k_h: f32) -> Result<()> {
        let pos = poses[0].encode(k_xy, k_h);
        let vel = poses[1].encode(k_xy, k_h);
        let acc = poses[2].encode(k_xy, k_h);
        let tx = [
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], vel[0], vel[1], vel[2], vel[3], vel[4],
            vel[5], acc[0], acc[1], acc[2], acc[3], acc[4], acc[5],
        ];
        self.write_regs(reg, &tx).await
    }

    pub async fn get_offsets(&mut self) -> Result<Pose> {
        self.read_pose(Register::OFFSETS, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }

    pub async fn set_offset(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::POS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }

    async fn check_product_id(&mut self) -> Result<()> {
        if self.read_reg(Register::PRODUCT_ID).await? == PRODUCT_ID {
            Ok(())
        } else {
            Err(Error::IncorrectProductID)
        }
    }

    pub async fn calibrate_imu(&mut self, n_samples: u8) -> Result<()> {
        self.write_reg(Register::IMU_CALIB, n_samples).await?;
        loop {
            self.wait_for_data().await?;
            if self.read_reg(Register::IMU_CALIB).await? == 0 {
                break Ok(());
            }
        }
    }

    pub async fn reset_tracking(&mut self) -> Result<()> {
        self.write_reg(Register::RESET, 0x01).await
    }

    pub async fn get_config(&mut self) -> Result<SignalProcessConfig> {
        Ok(SignalProcessConfig::from_bits(
            self.read_reg(Register::SIGNAL_PROCESS).await?,
        ))
    }

    pub async fn set_config(&mut self, config: &SignalProcessConfig) -> Result<()> {
        self.write_reg(Register::SIGNAL_PROCESS, config.into_bits())
            .await
    }

    pub async fn get_linear_scalar(&mut self) -> Result<f32> {
        self.read_scalar(Register::SCALAR_LINEAR).await
    }

    pub async fn set_linear_scalar(&mut self, scalar: f32) -> Result<()> {
        self.write_scalar(Register::SCALAR_LINEAR, scalar).await
    }

    pub async fn get_angular_scalar(&mut self) -> Result<f32> {
        self.read_scalar(Register::SCALAR_ANGULAR).await
    }

    pub async fn set_angular_scalar(&mut self, scalar: f32) -> Result<()> {
        self.write_scalar(Register::SCALAR_ANGULAR, scalar).await
    }

    async fn read_scalar(&mut self, reg: u8) -> Result<f32> {
        Ok((self.read_reg(reg).await? as i8) as f32 * 0.001 + 1.0)
    }

    async fn write_scalar(&mut self, reg: u8, scalar: f32) -> Result<()> {
        // bounds checking scalar
        if (MIN_SCALAR..=MAX_SCALAR).contains(&scalar) {
            // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
            self.write_reg(reg, (((scalar - 1.0) * 1000.0 + 0.5) as i8) as u8)
                .await
        } else {
            Err(Error::ScalarOutOfBounds)
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

    async fn write_regs(&mut self, reg: u8, tx: &[u8]) -> Result<()> {
        self.i2c
            .transaction(
                DEFAULT_ADDR,
                &mut [Operation::Write(&[reg]), Operation::Write(tx)],
            )
            .await
            .map_err(|_| Error::I2cError)
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<()> {
        self.i2c
            .write(DEFAULT_ADDR, &[reg, value])
            .await
            .map_err(|_| Error::I2cError)
    }
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub h: f32,
}

impl Pose {
    fn parse(rx: &[u8; 6], k_xy: f32, k_h: f32) -> Self {
        let x = i16::from_le_bytes([rx[0], rx[1]]) as f32 * k_xy;
        let y = i16::from_le_bytes([rx[2], rx[3]]) as f32 * k_xy;
        let h = i16::from_le_bytes([rx[4], rx[5]]) as f32 * k_h;
        Self { x, y, h }
    }
    fn encode(&self, k_xy: f32, k_h: f32) -> [u8; 6] {
        let x = ((self.x * k_xy) as i16).to_le_bytes();
        let y = ((self.y * k_xy) as i16).to_le_bytes();
        let h = ((self.h * k_h) as i16).to_le_bytes();
        [x[0], x[1], y[0], y[1], h[0], h[1]]
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Version {
    pub hw: u8,
    pub fw: u8,
}

/// Signal process config register bit fields
#[bitfield(u8)]
pub struct SignalProcessConfig {
    /// Whether to use the internal lookup table calibration for the
    /// optical sensor
    pub en_lut: bool,
    /// Whether to feed the accelerometer data to the Kalman filters
    pub en_acc: bool,
    /// Whether to rotate the IMU and optical sensor data by the
    /// heading angle
    pub en_rot: bool,
    /// Whether to use the correct sensor variance in the Kalman
    /// filters, or use 0 varaince to effectively disable the filters
    pub en_var: bool,
    #[bits(4)]
    __: u8,
}
