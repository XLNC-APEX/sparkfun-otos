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
use sparkfun_otos::{Pose, SparkFunOTOS};

async fn main() {
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
}
```
See [SparkFunOTOS](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.SparkFunOTOS.html) methods for more functionality

And [Pose](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.Pose.html) as main output type

Features
- defmt - implements `defmt::Format` from [defmt](https://crates.io/crates/defmt) crate for types.
- nalgebra - convert [Pose](https://docs.rs/sparkfun-otos/latest/sparkfun_otos/driver/otos/struct.Pose.html) into `Vector2<f32>`, `Point2<f32>`, `Isometry2<f32>` types from [nalgebra](https://crates.io/crates/nalgebra) crate

<!-- cargo-reedme: end -->
