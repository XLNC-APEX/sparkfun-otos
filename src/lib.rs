#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

pub mod driver;
pub mod error;
pub mod registers;

pub(crate) type Result<T> = core::result::Result<T, crate::error::Error>;

pub use otos::SparkfunOTOS;

use crate::driver::otos;

pub const PRODUCT_ID: u8 = 0x5F;
pub const DEFAULT_ADDR: u8 = 0x17;
