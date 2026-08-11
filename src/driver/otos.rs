use core::f32::consts::{FRAC_PI_2, PI, TAU};

use arrayref::array_ref;
use bitfield_struct::bitfield;
use embedded_hal_async::{
    digital::Wait,
    i2c::{I2c, Operation},
};
#[cfg(feature = "nalgebra")]
use nalgebra::{Isometry2, Point2, Vector2};

use crate::{
    DEFAULT_ADDR, K_I16_TO_METER, K_I16_TO_MPS, K_I16_TO_MPSS, K_I16_TO_RAD, K_I16_TO_RPS,
    K_I16_TO_RPSS, K_METER_TO_I16, K_MPS_TO_I16, K_MPSS_TO_I16, K_RAD_TO_I16, K_RPS_TO_I16,
    K_RPSS_TO_I16, MAX_SCALAR, MIN_SCALAR, PRODUCT_ID, Result, error::Error, registers::Register,
};

/// Main driver struct
pub struct SparkFunOTOS<I2C, IrqPin> {
    i2c: I2C,
    /// IO9 Input async pin(should not be pulled)
    irq_pin: IrqPin,
}

impl<I2C, IrqPin> SparkFunOTOS<I2C, IrqPin>
where
    I2C: I2c,
    IrqPin: Wait,
{
    /// params: i2c bus, IO9 Input async pin(should not be pulled)
    pub const fn new(i2c: I2C, irq_pin: IrqPin) -> Self {
        Self { i2c, irq_pin }
    }

    /// Checks product id
    pub async fn init(&mut self) -> Result<()> {
        self.check_product_id().await
    }

    /// Waits for data ready interrupt on IO9 pin, active low
    async fn wait_for_data(&mut self) -> Result<()> {
        self.irq_pin
            .wait_for_low()
            .await
            .map_err(|_| Error::PinError)
    }

    /// Get hardware and firmware versions
    pub async fn get_version(&mut self) -> Result<Versions> {
        let rx = self.read_regs::<2>(Register::HW_VERSION).await?;
        Ok(Versions {
            hw: Version::from_bits(rx[0]),
            fw: Version::from_bits(rx[1]),
        })
    }
    /// Get position
    pub async fn get_pos(&mut self) -> Result<Pose> {
        self.read_pose(Register::POS, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }
    /// Get velocity
    pub async fn get_vel(&mut self) -> Result<Pose> {
        self.read_pose(Register::VEL, K_I16_TO_MPS, K_I16_TO_RPS)
            .await
    }
    /// Get acceleration
    pub async fn get_acc(&mut self) -> Result<Pose> {
        self.read_pose(Register::ACCEL, K_I16_TO_MPSS, K_I16_TO_RPSS)
            .await
    }
    /// Get position, velocity at once.
    pub async fn get_pos_vel(&mut self) -> Result<[Pose; 2]> {
        self.read_2_poses(Register::POS).await
    }
    /// Get position, velocity, acceleration at once.
    pub async fn get_pos_vel_acc(&mut self) -> Result<[Pose; 3]> {
        self.read_poses(Register::POS).await
    }
    // Standard deviation
    /// Get standard deviation of position.
    pub async fn get_pos_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::POS_SD, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }
    /// Get standard deviation of velocity.
    pub async fn get_vel_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::VEL_SD, K_I16_TO_MPS, K_I16_TO_RPS)
            .await
    }
    /// Get standard deviation of acceleration.
    pub async fn get_acc_sd(&mut self) -> Result<Pose> {
        self.read_pose(Register::ACCEL_SD, K_I16_TO_MPSS, K_I16_TO_RPSS)
            .await
    }
    /// Get standard deviation of position, velocity at once.
    pub async fn get_pos_vel_sd(&mut self) -> Result<[Pose; 2]> {
        self.read_2_poses(Register::POS_SD).await
    }
    /// Get standard deviation of position, velocity, acceleration at once.
    pub async fn get_pos_vel_acc_sd(&mut self) -> Result<[Pose; 3]> {
        self.read_poses(Register::POS_SD).await
    }

    async fn read_pose(&mut self, reg: u8, k_xy: f32, k_h: f32) -> Result<Pose> {
        self.wait_for_data().await?;
        let rx = self.read_regs::<6>(reg).await?;
        Ok(Pose::parse(&rx, k_xy, k_h))
    }

    async fn read_2_poses(&mut self, reg: u8) -> Result<[Pose; 2]> {
        self.wait_for_data().await?;
        let rx = self.read_regs::<12>(reg).await?;
        Ok([
            Pose::parse(array_ref![rx, 0, 6], K_I16_TO_METER, K_I16_TO_RAD),
            Pose::parse(array_ref![rx, 6, 6], K_I16_TO_METER, K_I16_TO_RAD),
        ])
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
    /// Set position
    pub async fn set_pos(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::POS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }
    /// Set velocity
    pub async fn set_vel(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::VEL, K_MPS_TO_I16, K_RPS_TO_I16)
            .await
    }
    /// Set acceleration
    pub async fn set_acc(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::ACCEL, K_MPSS_TO_I16, K_RPSS_TO_I16)
            .await
    }
    /// Set position, velocity, acceleration at once.
    pub async fn set_pos_vel_acc(&mut self, poses: &[Pose; 3]) -> Result<()> {
        self.write_poses(poses, Register::POS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }
    // Standard deviation.
    /// Set standard deviation of position.
    pub async fn set_pos_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::POS_SD, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }
    /// Set standard deviation of velocity.
    pub async fn set_vel_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::VEL_SD, K_MPS_TO_I16, K_RPS_TO_I16)
            .await
    }
    /// Set standard deviation of acceleration.
    pub async fn set_acc_sd(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::ACCEL_SD, K_MPSS_TO_I16, K_RPSS_TO_I16)
            .await
    }
    /// Set standard deviation of position, velocity, acceleration at once.
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
    /// Get currently set OTOS offset. Offset of OTOS is from perspective of desired tracking pose(ex. robot center)
    /// RF coordinate system(+X is Right,+Y is Forward) by default.
    pub async fn get_offsets(&mut self) -> Result<Pose> {
        self.read_pose(Register::OFFSETS, K_I16_TO_METER, K_I16_TO_RAD)
            .await
    }
    /// Set OTOS offset. Offset of OTOS is from perspective of desired tracking pose(ex. robot center)
    /// RF coordinate system(+X is Right,+Y is Forward) by default.
    ///
    /// Output coord. system can be changed, ex. FL, see [Self::set_offset_fl] and others
    /// Coord. systems can be only rotations of RF. supported currently: (RF, FL, LD, DR)
    /// The conversion is happening on the OTOS. We abuse offset properties.
    pub async fn set_offset(&mut self, pose: &Pose) -> Result<()> {
        self.write_pose(pose, Register::OFFSETS, K_METER_TO_I16, K_RAD_TO_I16)
            .await
    }
    /// Same input as [Self::set_offset] however changes output coordinate system to FL(X forward, Y left)
    pub async fn set_offset_fl(&mut self, pose: &Pose) -> Result<()> {
        // FL is 90deg ccw rotation from RF. To change CS rotate offset to inverse angle -90deg.
        let mut rotated_pose = Pose::new(pose.y, -pose.x, pose.h - FRAC_PI_2);
        // Make sure heading is valid, in range [-pi, pi)
        rotated_pose.wrap_heading();
        self.set_offset(&rotated_pose).await
    }
    /// Same input as [Self::set_offset] however changes output coordinate system to LD(X left, Y down)
    pub async fn set_offset_ld(&mut self, pose: &Pose) -> Result<()> {
        // LD is 180deg ccw rotation from RF. To change CS rotate offset to inverse angle -180deg.
        let mut rotated_pose = Pose::new(-pose.x, -pose.y, pose.h - PI);
        // Make sure heading is valid, in range [-pi, pi)
        rotated_pose.wrap_heading();
        self.set_offset(&rotated_pose).await
    }
    /// Same input as [Self::set_offset] however changes output coordinate system to DR(X down, Y right)
    pub async fn set_offset_dr(&mut self, pose: &Pose) -> Result<()> {
        // FL is -90deg ccw rotation from RF. To change CS rotate offset to inverse angle 90deg.
        let mut rotated_pose = Pose::new(-pose.y, pose.x, pose.h + FRAC_PI_2);
        // Make sure heading is valid, in range [-pi, pi)
        rotated_pose.wrap_heading();
        self.set_offset(&rotated_pose).await
    }
    /// Checks if product id equals [PRODUCT_ID]
    async fn check_product_id(&mut self) -> Result<()> {
        if self.read_reg(Register::PRODUCT_ID).await? == PRODUCT_ID {
            Ok(())
        } else {
            Err(Error::IncorrectProductID)
        }
    }

    /// Calibrates IMU, resets the velocity, acceleration.
    /// OTOS should be completely still during calibration.
    /// n_samples - Number of samples taken for calibration, max = 255.
    /// 1 sample takes 2.4ms approximately.
    pub async fn calibrate_imu(&mut self, n_samples: u8) -> Result<()> {
        self.write_reg(Register::IMU_CALIB, n_samples).await?;
        loop {
            self.wait_for_data().await?;
            if self.read_reg(Register::IMU_CALIB).await? == 0 {
                break Ok(());
            }
        }
    }

    /// Resets position to origin(0,0).
    pub async fn reset_tracking(&mut self) -> Result<()> {
        self.write_reg(Register::RESET, 0x01).await?;
        // Discard leftover old pos. After it pos is 0. Tested on hw.
        // TODO: do other methods require discarding too?
        self.get_pos().await?;
        Ok(())
    }

    /// Gets [SignalProcessConfig] from OTOS.
    pub async fn get_config(&mut self) -> Result<SignalProcessConfig> {
        Ok(SignalProcessConfig::from_bits(
            self.read_reg(Register::SIGNAL_PROCESS).await?,
        ))
    }

    /// Sets [SignalProcessConfig] from OTOS.
    pub async fn set_config(&mut self, config: &SignalProcessConfig) -> Result<()> {
        self.write_reg(Register::SIGNAL_PROCESS, config.into_bits())
            .await
    }

    /// Gets linear scalar, coefficient of position. Used by OTOS.
    /// Scalar is between 0.872 and 1.127
    pub async fn get_linear_scalar(&mut self) -> Result<f32> {
        self.read_scalar(Register::SCALAR_LINEAR).await
    }

    /// Sets linear scalar, coefficient of position. Used by OTOS.
    /// Scalar must be between 0.872 and 1.127
    pub async fn set_linear_scalar(&mut self, scalar: f32) -> Result<()> {
        self.write_scalar(Register::SCALAR_LINEAR, scalar).await
    }

    /// Gets angular scalar, coefficient of heading. Used by OTOS.
    /// Scalar is between 0.872 and 1.127
    pub async fn get_angular_scalar(&mut self) -> Result<f32> {
        self.read_scalar(Register::SCALAR_ANGULAR).await
    }

    /// Gets angular scalar, coefficient of heading. Used by OTOS.
    /// Scalar must be between 0.872 and 1.127
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

    // TODO: Does it need after this read leftover pos and discard it?
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<()> {
        self.i2c
            .write(DEFAULT_ADDR, &[reg, value])
            .await
            .map_err(|_| Error::I2cError)
    }
}
/// Similar to `Vector2<f32>` with additional heading(or it's derivatives).
/// Usually represents: position, velocity, acceleration, offset.
/// Can be converted into `Vector2<f32>`, `Point2<f32>`, `Isometry2<f32>` (uses heading) with `nalgebra` feature.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    /// Heading, in radians
    pub h: f32,
}

impl Pose {
    /// Param units: x: meters, y: meters, h: radians
    pub const fn new(x: f32, y: f32, h: f32) -> Self {
        Self { x, y, h }
    }
    /// Same as [Self::new] but x, y are in millimeters.
    pub const fn new_mm(x: f32, y: f32, h: f32) -> Self {
        Self {
            x: x / 1000.0,
            y: y / 1000.0,
            h,
        }
    }
    const fn parse(rx: &[u8; 6], k_xy: f32, k_h: f32) -> Self {
        let x = i16::from_le_bytes([rx[0], rx[1]]) as f32 * k_xy;
        let y = i16::from_le_bytes([rx[2], rx[3]]) as f32 * k_xy;
        let h = i16::from_le_bytes([rx[4], rx[5]]) as f32 * k_h;
        Self { x, y, h }
    }
    const fn encode(&self, k_xy: f32, k_h: f32) -> [u8; 6] {
        let x = ((self.x * k_xy) as i16).to_le_bytes();
        let y = ((self.y * k_xy) as i16).to_le_bytes();
        let h = ((self.h * k_h) as i16).to_le_bytes();
        [x[0], x[1], y[0], y[1], h[0], h[1]]
    }
    /// Wraps heading to [-pi,pi)
    /// Sometimes needed after heading changing operations.
    pub const fn wrap_heading(&mut self) {
        if self.h >= PI {
            self.h -= TAU;
        }
        if self.h < -PI {
            self.h += TAU;
        }
    }
}

#[cfg(feature = "nalgebra")]
impl From<Pose> for Point2<f32> {
    fn from(pose: Pose) -> Self {
        Point2::new(pose.x, pose.y)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Point2<f32>> for Pose {
    fn from(v: Point2<f32>) -> Self {
        Pose::new(v.x, v.y, 0.0)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Pose> for Vector2<f32> {
    fn from(pose: Pose) -> Self {
        Vector2::new(pose.x, pose.y)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Vector2<f32>> for Pose {
    fn from(v: Vector2<f32>) -> Self {
        Pose::new(v.x, v.y, 0.0)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Pose> for Isometry2<f32> {
    fn from(pose: Pose) -> Self {
        Isometry2::new(Vector2::new(pose.x, pose.y), pose.h)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Isometry2<f32>> for Pose {
    fn from(v: Isometry2<f32>) -> Self {
        Pose::new(v.translation.x, v.translation.y, v.rotation.angle())
    }
}

use core::ops::{Add, Div, Mul, Sub};

#[cfg(feature = "nalgebra")]
impl Add<Vector2<f32>> for Pose {
    type Output = Self;
    fn add(self, rhs: Vector2<f32>) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            h: self.h,
        }
    }
}

#[cfg(feature = "nalgebra")]
impl Sub<Vector2<f32>> for Pose {
    type Output = Self;
    fn sub(self, rhs: Vector2<f32>) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            h: self.h,
        }
    }
}

impl Add<Self> for Pose {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            h: self.h + rhs.h,
        }
    }
}

impl Sub<Self> for Pose {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            h: self.h - rhs.h,
        }
    }
}

impl Mul<f32> for Pose {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            h: self.h,
        }
    }
}

impl Div<f32> for Pose {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            h: self.h,
        }
    }
}

/// Hardware, Firmware versions struct.
/// Supports `defmt` with `defmt` feature
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Versions {
    pub hw: Version,
    pub fw: Version,
}

/// Signal process config register bit fields
#[cfg_attr(feature = "defmt", bitfield(u8, defmt = true, default = false))]
#[cfg_attr(not(feature = "defmt"), bitfield(u8, default = false))]
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

impl Default for SignalProcessConfig {
    fn default() -> Self {
        Self(0x0F)
    }
}

/// bitfield struct of Hardware/Firmware version.
/// Has minor, major 4bit fields.
/// Supports `defmt` with `defmt` feature
#[bitfield(u8)]
#[derive(PartialEq)]
pub struct Version {
    #[bits(4)]
    pub minor: u8,
    #[bits(4)]
    pub major: u8,
}
#[cfg(feature = "defmt")]
impl defmt::Format for Version {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}", self.major(), self.minor(),)
    }
}
