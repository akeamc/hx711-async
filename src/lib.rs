#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

//! Another HX711 driver for embedded systems. This one is asynchronous and
//! instead of polling the data pin, it waits for it to go low before reading.
//! This should hopefully compensate for the bit-banging.
//!
//! This driver draws inspiration from another HX711 driver, [`loadcell`](https://crates.io/crates/loadcell).

use critical_section::CriticalSection;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

/// The time between SCK edges should, according to the datasheet, be at least
/// 200 ns and at most 50 μs, and ideally 1 μs.
const DELAY_TIME_NS: u32 = 1000;

/// The input and gain is selected with this one enum.
#[repr(u8)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Channel A, gain 128.
    A128 = 1,
    /// Channel B, gain 32.
    B32 = 2,
    /// Channel B, gain 64.
    B64 = 3,
}

/// An HX711 instance.
pub struct Hx711<SckPin, DataPin, Delay> {
    sck: SckPin,
    data: DataPin,
    delay: Delay,
    mode: Mode,
}

impl<SckPin, DataPin, Delay, Error> Hx711<SckPin, DataPin, Delay>
where
    SckPin: OutputPin<Error = Error>,
    DataPin: InputPin<Error = Error> + Wait<Error = Error>,
    Delay: embedded_hal::delay::DelayNs,
{
    /// Create a new HX711 driver.
    pub const fn new(sck: SckPin, data: DataPin, delay: Delay, mode: Mode) -> Self {
        Self {
            sck,
            data,
            delay,
            mode,
        }
    }

    /// Wait until a value is available and read it.
    pub async fn read(&mut self) -> Result<i32, Error> {
        self.data.wait_for_low().await?;

        // Because timing is everything, we cannot risk interrupts during the
        // conversion. So no async delay is used.
        let mut bits = critical_section::with(|cs| {
            let bits = self.read_bits(cs)?;
            self.toggle_mode_bits(cs)?;
            Ok(bits)
        })?;

        // convert 24-bit two's complement to 32-bit two's complement
        if bits & 0x800000 != 0 {
            bits |= 0xff000000;
        }

        Ok(bits.cast_signed())
    }

    /// Set the gain and input for the next reading.
    pub const fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    fn read_bits(&mut self, cs: CriticalSection) -> Result<u32, Error> {
        let mut value = 0u32;
        for _ in 0..24 {
            // msb first
            value <<= 1;
            value |= u32::from(self.read_bit(cs)?);
        }
        Ok(value)
    }

    fn toggle_mode_bits(&mut self, cs: CriticalSection) -> Result<(), Error> {
        for _ in 0..self.mode as u8 {
            // toggle SCK
            let _ = self.read_bit(cs)?;
        }
        Ok(())
    }

    fn read_bit(&mut self, _cs: CriticalSection) -> Result<bool, Error> {
        self.sck.set_high()?;
        self.delay.delay_ns(DELAY_TIME_NS);

        let bit = self.data.is_high()?;

        self.sck.set_low()?;
        self.delay.delay_ns(DELAY_TIME_NS);

        Ok(bit)
    }
}
