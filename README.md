<!-- cargo-reedme: start -->

<!-- cargo-reedme: info-start

    Do not edit this region by hand
    ===============================

    This region was generated from Rust documentation comments by `cargo-reedme` using this command:

        cargo +nightly reedme

    for more info: https://github.com/nik-rev/cargo-reedme

cargo-reedme: info-end -->

SparkFun OTOS [embedded_hal_async](https://docs.rs/embedded_hal_async/latest/embedded_hal_async/) driver

Example
```rust
use core::f32::consts::FRAC_PI_2;
use sparkfun_otos::{Pose, SparkFunOTOS};

async fn main() {
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
}
```
See [SparkFunOTOS](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.SparkFunOTOS.html) methods for more functionality

And [Pose](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.Pose.html) as main output type

Features
- defmt - implements `defmt::Format` from [defmt](https://crates.io/crates/defmt) crate for types.
- nalgebra - convert [Pose](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.Pose.html) into `Vector2<f32>`, `Point2<f32>`, `Isometry2<f32>` types from [nalgebra](https://crates.io/crates/nalgebra) crate

<!-- cargo-reedme: end -->
