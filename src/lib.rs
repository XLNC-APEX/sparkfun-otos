#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

//! SparkFun OTOS [embedded_hal_async] driver
//!
//! Example
//! ```
#![doc = doctest_file::include_doctest!("examples/basic_use.rs")]
//! ```

pub mod driver;
pub mod error;
pub mod registers;

pub(crate) type Result<T> = core::result::Result<T, crate::error::Error>;

pub use otos::Pose;
pub use otos::SparkFunOTOS;
pub use otos::Versions;

use crate::driver::otos;

// The official Arduino library: github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library
// which is licensed under MIT license by Sparkfun Electronics
// has been used a reference for writing this driver,
// some code portions were copied and modified like registers,
// some were copied as is, like some documentation comments.
// Thank you Sparkfun Electronics!
// You can find the notice in third-party-license-notice.md file
// in root directory of this repository.
// TODO: License properly?

pub const PRODUCT_ID: u8 = 0x5F;
pub const DEFAULT_ADDR: u8 = 0x17;

const K_DEGREE_TO_RADIAN: f32 = PI / 180.0;

/// Conversion factor for the linear position registers. 16-bit signed
/// registers with a max value of 10 meters (394 inches) gives a resolution
/// of about 0.0003 mps (0.012 ips)
const K_METER_TO_I16: f32 = 32768.0 / 10.0;
const K_I16_TO_METER: f32 = 1.0 / K_METER_TO_I16;

// Conversion factor for the linear velocity registers. 16-bit signed
// registers with a max value of 5 mps (197 ips) gives a resolution of about
// 0.00015 mps (0.006 ips)
const K_MPS_TO_I16: f32 = 32768.0 / 5.0;
const K_I16_TO_MPS: f32 = 1.0 / K_MPS_TO_I16;

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
const K_MPSS_TO_I16: f32 = 32768.0 / (16.0 * 9.80665);
const K_I16_TO_MPSS: f32 = 1.0 / K_MPSS_TO_I16;

use core::f32::consts::PI;

/// Conversion factor for the angular position registers. 16-bit signed
/// registers with a max value of pi radians (180 degrees) gives a resolution
/// of about 0.00096 radians (0.0055 degrees)
const K_RAD_TO_I16: f32 = 32768.0 / PI;
const K_I16_TO_RAD: f32 = 1.0 / K_RAD_TO_I16;

// Conversion factor for the angular velocity registers. 16-bit signed
// registers with a max value of 34.9 rps (2000 dps) gives a resolution of
// about 0.0011 rps (0.061 degrees per second)
const K_RPS_TO_I16: f32 = 32768.0 / (2000.0 * K_DEGREE_TO_RADIAN);
const K_I16_TO_RPS: f32 = 1.0 / K_RPS_TO_I16;

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a
// resolution of about 0.096 rps^2 (5.5 dps^2)
const K_RPSS_TO_I16: f32 = 32768.0 / (PI * 1000.0);
const K_I16_TO_RPSS: f32 = 1.0 / K_RPSS_TO_I16;

const MIN_SCALAR: f32 = 0.872;
const MAX_SCALAR: f32 = 1.127;
