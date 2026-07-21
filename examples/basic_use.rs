use embedded_hal_mock::eh1::digital::State; //
use sparkfun_otos::{DEFAULT_ADDR, registers::Register}; //
use sparkfun_otos::{Pose, SparkFunOTOS};

#[tokio::main] //
async fn main() {
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, Transaction as PinTransaction}; //
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction}; //
    #[rustfmt::skip] // Prevents rustfmt from moving '//' suffix on next line //
    let i2c_expect = [ //
        I2cTransaction::write(DEFAULT_ADDR, vec![Register::IMU_CALIB, 255]), // .calibrate_imu(255) //
        I2cTransaction::write_read(DEFAULT_ADDR, vec![Register::IMU_CALIB], vec![0]), //
        I2cTransaction::write(DEFAULT_ADDR, vec![Register::RESET, 0x01]), // .reset_tracking() //
        I2cTransaction::write_read(DEFAULT_ADDR, vec![Register::POS], vec![0u8; 6]), // .get_pos() //
    ]; //
    #[rustfmt::skip] //
    let pin_expect = [ //
        PinTransaction::wait_for_state(State::Low), //
        PinTransaction::wait_for_state(State::Low), //
    ]; //
    let mut i2c_1 = I2cMock::new(&i2c_expect); //
    let mut irq_pin_1 = PinMock::new(&pin_expect); //
    let i2c = i2c_1.clone(); //
    let irq_pin = irq_pin_1.clone(); //
    // Params: i2c bus, async Input pin(impls async Wait trait) connected to IO9 on OTOS.
    let mut otos = SparkFunOTOS::new(i2c, irq_pin);
    otos.calibrate_imu(255).await.unwrap();
    otos.reset_tracking().await.unwrap();
    let pos = otos.get_pos().await.unwrap();

    assert_eq!(
        pos,
        Pose {
            x: 0.0,
            y: 0.0,
            h: 0.0
        }
    );
    i2c_1.done(); // TODO: Make such test on real hardware. I am not sure pos will be 0, //
    irq_pin_1.done(); // it should but bugs might be there. // 
}
