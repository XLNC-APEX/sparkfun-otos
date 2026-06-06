#[allow(dead_code)]
pub struct Register;

#[allow(dead_code)]
impl Register {
    pub const PRODUCT_ID: u8 = 0x00;

    pub const VERSIONS: u8 = Self::HW_VERSION;
    pub const HW_VERSION: u8 = 0x01;
    pub const FW_VERSION: u8 = 0x02;

    pub const SCALARS: u8 = Self::SCALAR_LINEAR;
    pub const SCALAR_LINEAR: u8 = 0x04;
    pub const SCALAR_ANGULAR: u8 = 0x05;

    pub const IMU_CALIB: u8 = 0x06;
    pub const RESET: u8 = 0x07;
    pub const SIGNAL_PROCESS: u8 = 0x0E;
    pub const SELF_TEST: u8 = 0x0F;

    pub const OFFSETS: u8 = 0x10;

    pub const POS: u8 = Self::POS_X_L;

    pub const POS_X_L: u8 = 0x20;
    pub const POS_X_H: u8 = 0x21;
    pub const POS_Y_L: u8 = 0x22;
    pub const POS_Y_H: u8 = 0x23;
    pub const POS_HEADING_L: u8 = 0x24;
    pub const POS_HEADING_H: u8 = 0x25;

    pub const VEL: u8 = Self::VEL_X_L;

    pub const VEL_X_L: u8 = 0x26;
    pub const VEL_X_H: u8 = 0x27;
    pub const VEL_Y_L: u8 = 0x28;
    pub const VEL_Y_H: u8 = 0x29;
    pub const VEL_HEADING_L: u8 = 0x2A;
    pub const VEL_HEADING_H: u8 = 0x2B;

    pub const ACCEL: u8 = Self::ACCEL_X_L;

    pub const ACCEL_X_L: u8 = 0x2C;
    pub const ACCEL_X_H: u8 = 0x2D;
    pub const ACCEL_Y_L: u8 = 0x2E;
    pub const ACCEL_Y_H: u8 = 0x2F;
    pub const ACCEL_HEADING_L: u8 = 0x30;
    pub const ACCEL_HEADING_H: u8 = 0x31;

    pub const POS_SD: u8 = Self::POS_X_L_SD;

    pub const POS_X_L_SD: u8 = 0x32;
    pub const POS_X_H_SD: u8 = 0x33;
    pub const POS_Y_L_SD: u8 = 0x34;
    pub const POS_Y_H_SD: u8 = 0x35;
    pub const POS_HEADING_L_SD: u8 = 0x36;
    pub const POS_HEADING_H_SD: u8 = 0x37;

    pub const VEL_SD: u8 = Self::VEL_X_L_SD;

    pub const VEL_X_L_SD: u8 = 0x38;
    pub const VEL_X_H_SD: u8 = 0x39;
    pub const VEL_Y_L_SD: u8 = 0x3A;
    pub const VEL_Y_H_SD: u8 = 0x3B;
    pub const VEL_HEADING_L_SD: u8 = 0x3C;
    pub const VEL_HEADING_H_SD: u8 = 0x3D;

    pub const ACCEL_SD: u8 = Self::ACCEL_X_L_SD;

    pub const ACCEL_X_L_SD: u8 = 0x3E;
    pub const ACCEL_X_H_SD: u8 = 0x3F;
    pub const ACCEL_Y_L_SD: u8 = 0x40;
    pub const ACCEL_Y_H_SD: u8 = 0x41;
    pub const ACCEL_HEADING_L_SD: u8 = 0x42;
    pub const ACCEL_HEADING_H_SD: u8 = 0x43;
}
