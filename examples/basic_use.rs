use sparkfun_otos::SparkFunOTOS;

#[tokio::main] //
async fn main() {
    use embedded_hal_mock::eh1::digital::Mock as InputMock; //
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction}; //
    #[rustfmt::skip] //
    let _e = [ //
        I2cTransaction::write(0xaa, vec![1, 2]), //
        I2cTransaction::read(0xbb, vec![3, 4]),  //
    ]; //
    let mut i2c_1 = I2cMock::new(&[]); //
    let mut irq_pin_1 = InputMock::new(&[]); //
    let i2c = i2c_1.clone(); //
    let irq_pin = irq_pin_1.clone(); //
    let mut otos = SparkFunOTOS::new(i2c, irq_pin);
    otos.calibrate_imu(255).await.unwrap();
    otos.reset_tracking().await.unwrap();
    let pos = otos.get_pos().await.unwrap();
    println!("{:?}", pos);
    i2c_1.done(); //
    irq_pin_1.done(); //
}
