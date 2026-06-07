#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

pub mod driver;
pub mod error;
pub mod registers;

pub(crate) type Result<T> = core::result::Result<T, crate::error::Error>;

pub use otos::SparkfunOTOS;

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
