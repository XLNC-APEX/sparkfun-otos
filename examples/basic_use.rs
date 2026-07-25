use core::f32::consts::FRAC_PI_2;
use embedded_hal_mock::eh1::digital::State; //
use sparkfun_otos::{DEFAULT_ADDR, registers::Register}; //
use sparkfun_otos::{Pose, SparkFunOTOS};

#[tokio::main] //
async fn main() {
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, Transaction as PinTransaction}; //
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction}; //
    #[rustfmt::skip] // Prevents rustfmt from moving '//' suffix on next line //
    let i2c_expect = [ //
        I2cTransaction::transaction_start(DEFAULT_ADDR), // .set_offset(&offset) //
        I2cTransaction::write(DEFAULT_ADDR, vec![Register::OFFSETS]), //
        I2cTransaction::write(DEFAULT_ADDR, vec![0,0,178,0,0,64]), //
        I2cTransaction::transaction_end(DEFAULT_ADDR), //
        I2cTransaction::write(DEFAULT_ADDR, vec![Register::IMU_CALIB, 255]), // .calibrate_imu(255) //
        I2cTransaction::write_read(DEFAULT_ADDR, vec![Register::IMU_CALIB], vec![0]), //
        I2cTransaction::write(DEFAULT_ADDR, vec![Register::RESET, 0x01]), // .reset_tracking() //
        I2cTransaction::write_read(DEFAULT_ADDR, vec![Register::POS], vec![0xbbu8; 6]), // discarding leftover: .get_pos() //
        I2cTransaction::write_read(DEFAULT_ADDR, vec![Register::POS], vec![0u8; 6]), // .get_pos() //
    ]; //
    #[rustfmt::skip] //
    let pin_expect = [ //
        PinTransaction::wait_for_state(State::Low), // .calibrate_imu(255) //
        PinTransaction::wait_for_state(State::Low), // discarding: .get_pos() //
        PinTransaction::wait_for_state(State::Low), // .get_pos( //
    ]; //
    let mut i2c_1 = I2cMock::new(&i2c_expect); //
    let mut irq_pin_1 = PinMock::new(&pin_expect); //
    let i2c = i2c_1.clone(); //
    let irq_pin = irq_pin_1.clone(); //
    // Params: i2c bus, async Input pin(impls async Wait trait) connected to IO9 on OTOS.
    let mut otos = SparkFunOTOS::new(i2c, irq_pin);
    // Offset is used to compensate for offset placement
    // of OTOS relative to desired center pose of tracking(ex. robot center).
    // For offset the coordinate system is:
    // +X is Right, +Y is Forward. (RF, Math like)
    // For example if from the desired pose perspective the otos is placed:
    // 54.5 mm forward (Y), rotated 90deg ccw (== PI/2)
    // Correcting offset is:
    let offset = Pose::new_mm(0.0, 54.5, FRAC_PI_2);
    otos.set_offset(&offset).await.unwrap();
    // Output coordinate system can be changed, see [Pose::set_offset]

    // Resets the velocity, acceleration. OTOS should not be moving!
    otos.calibrate_imu(255).await.unwrap();
    // Resets pos to 0
    otos.reset_tracking().await.unwrap();
    let pos = otos.get_pos().await.unwrap();

    // After reset, position should be 0 or very close to 0
    assert_eq!(pos, Pose::new(0.0, 0.0, 0.0));
    i2c_1.done(); // TODO: Make such test on real hardware. I am not sure pos will be 0, //
    irq_pin_1.done(); // it should but bugs might be there. // 
}
