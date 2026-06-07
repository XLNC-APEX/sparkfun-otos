/// Possible errors from Sparkfun OTOS
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    Timeout,
    I2cError,
    PinError,
    IncorrectProductID,
    CalibrationError,
    ScalarOutOfBounds,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::I2cError => write!(f, "I2C communication error"),
            Self::PinError => write!(f, "Interrurpt waiting error"),
            _ => Ok(()), // wtf did I do with this line?
        }
    }
}
