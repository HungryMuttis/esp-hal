#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # LED Controller (LEDC)
//!
//! ## Overview
//!
//! The LEDC peripheral is primarily designed to control the intensity of LEDs,
//! although it can also be used to generate PWM signals for other purposes. It
//! has multiple channels which can generate independent waveforms that can be
//! used, for example, to drive RGB LED devices.
//!
//! The PWM controller can automatically increase or decrease the duty cycle
//! gradually, allowing for fades without any processor interference.
//!
//! ## Configuration
//! Currently only supports fixed-frequency output. High Speed channels are
//! available for the ESP32 only, while Low Speed channels are available for all
//! supported chips.
//!
//! ## Examples
//!
//! ### Low Speed Channel
//!
//! The following example will configure the Low Speed Channel0 to 24kHz output
//! with 10% duty using the ABPClock and turn on LED with the option to change
//! LED intensity depending on `duty` value. Possible values (`u32`) are in
//! range 0..100.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::ledc::Ledc;
//! # use esp_hal::ledc::LSGlobalClkSource;
//! # use esp_hal::ledc::timer::{self, TimerSpeed};
//! # use esp_hal::ledc::LowSpeed;
//! # use esp_hal::ledc::channel::{self};
//! # use esp_hal::gpio::DriveMode;
//! # use esp_hal::time::Rate;
//! # let led_pin = peripherals.GPIO0;
//!
//! // Create a new Ledc driver and initialize global slow clock.
//! let mut ledc = Ledc::new(peripherals.LEDC, LSGlobalClkSource::APBClk);
//!
//! // Initialize a new timer.
//! let timer0 = ledc.timer0.configure(timer::Config {
//!     duty: timer::Duty::Bit10,
//!     clock_source: timer::ClockSource::APBClk,
//!     frequency: Rate::from_khz(24),
//! })?;
//!
//! // Initialize a new channel with the timer.
//! let mut channel0 = ledc.channel0.configure(
//!     channel::Config {
//!         timer: &timer0,
//!         duty: 0, // fully off
//!         pin_config: DriveMode::PushPull,
//!     },
//!     led_pin,
//! )?;
//!
//! // Get the duty value to set the light to 100%
//! let max_duty = channel0.percent_to_duty(100);
//!
//! loop {
//!     // Set up a breathing LED: fade from off to on over a second, then
//!     // from on back off over the next second. Then loop.
//!     channel0.start_duty_fade(0, max_duty, 1000).unwrap();
//!     while channel0.is_duty_fade_running() {}
//!     channel0.start_duty_fade(max_duty, 0, 1000).unwrap();
//!     while channel0.is_duty_fade_running() {}
//! }
//! # }
//! ```
//!
//! ## Implementation State
//! - Source clock selection is not supported
//! - Interrupts are not supported

use crate::{
    peripherals::LEDC,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub mod channel;
mod low_level;
pub mod timer;

/// Global slow clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSGlobalClkSource {
    /// APB clock.
    APBClk,
}

#[cfg(ledc_version = "1")]
#[derive(Clone, Copy)]
/// Used to specify HighSpeed Timer/Channel
pub struct HighSpeed {}

#[derive(Clone, Copy)]
/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

/// Trait representing the speed mode of a clock or peripheral.
pub trait Speed {
    /// Boolean constant indicating whether the speed is high-speed.
    const IS_HS: bool;
}

#[cfg(ledc_version = "1")]
impl Speed for HighSpeed {
    const IS_HS: bool = true;
}

impl Speed for LowSpeed {
    const IS_HS: bool = false;
}

/// LEDC (LED PWM Controller)
pub struct Ledc<'d> {
    _instance: LEDC<'d>,
    /// Low Speed Timer 0
    pub timer0: timer::Timer<LowSpeed>,
    /// Low Speed Timer 1
    pub timer1: timer::Timer<LowSpeed>,
    /// Low Speed Timer 2
    pub timer2: timer::Timer<LowSpeed>,
    /// Low Speed Timer 3
    pub timer3: timer::Timer<LowSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 0
    pub hs_timer0: timer::Timer<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 1
    pub hs_timer1: timer::Timer<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 2
    pub hs_timer2: timer::Timer<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 3
    pub hs_timer3: timer::Timer<HighSpeed>,

    /// Low Speed Channel 0
    pub channel0: channel::Channel<LowSpeed>,
    /// Low Speed Channel 1
    pub channel1: channel::Channel<LowSpeed>,
    /// Low Speed Channel 2
    pub channel2: channel::Channel<LowSpeed>,
    /// Low Speed Channel 3
    pub channel3: channel::Channel<LowSpeed>,
    /// Low Speed Channel 4
    pub channel4: channel::Channel<LowSpeed>,
    /// Low Speed Channel 5
    pub channel5: channel::Channel<LowSpeed>,
    #[cfg(ledc_channel_count = "8")]
    /// Low Speed Channel 6
    pub channel6: channel::Channel<LowSpeed>,
    #[cfg(ledc_channel_count = "8")]
    /// Low Speed Channel 7
    pub channel7: channel::Channel<LowSpeed>,

    #[cfg(ledc_version = "1")]
    /// High Speed Channel 0
    pub hs_channel0: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 1
    pub hs_channel1: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 2
    pub hs_channel2: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 3
    pub hs_channel3: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 4
    pub hs_channel4: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 5
    pub hs_channel5: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 6
    pub hs_channel6: channel::Channel<HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 7
    pub hs_channel7: channel::Channel<HighSpeed>,
}

impl<'d> Ledc<'d> {
    /// Return a new LEDC
    pub fn new(_instance: LEDC<'d>, clock_source: LSGlobalClkSource) -> Self {
        if PeripheralClockControl::enable(PeripheralEnable::Ledc) {
            PeripheralClockControl::reset(PeripheralEnable::Ledc);
        } else {
            // Refcount was more than 0. Decrement to avoid overflow because we don't handle
            // dropping the driver.
            PeripheralClockControl::disable(PeripheralEnable::Ledc);
        }
        low_level::set_global_slow_clock(LEDC::regs(), clock_source);

        Ledc {
            _instance,
            timer0: timer::Timer::new(timer::Number::Timer0),
            timer1: timer::Timer::new(timer::Number::Timer1),
            timer2: timer::Timer::new(timer::Number::Timer2),
            timer3: timer::Timer::new(timer::Number::Timer3),
            #[cfg(ledc_version = "1")]
            hs_timer0: timer::Timer::new(timer::Number::Timer0),
            #[cfg(ledc_version = "1")]
            hs_timer1: timer::Timer::new(timer::Number::Timer1),
            #[cfg(ledc_version = "1")]
            hs_timer2: timer::Timer::new(timer::Number::Timer2),
            #[cfg(ledc_version = "1")]
            hs_timer3: timer::Timer::new(timer::Number::Timer3),
            channel0: channel::Channel::new(channel::Number::Channel0),
            channel1: channel::Channel::new(channel::Number::Channel1),
            channel2: channel::Channel::new(channel::Number::Channel2),
            channel3: channel::Channel::new(channel::Number::Channel3),
            channel4: channel::Channel::new(channel::Number::Channel4),
            channel5: channel::Channel::new(channel::Number::Channel5),
            #[cfg(ledc_channel_count = "8")]
            channel6: channel::Channel::new(channel::Number::Channel6),
            #[cfg(ledc_channel_count = "8")]
            channel7: channel::Channel::new(channel::Number::Channel7),
            #[cfg(ledc_version = "1")]
            hs_channel0: channel::Channel::new(channel::Number::Channel0),
            #[cfg(ledc_version = "1")]
            hs_channel1: channel::Channel::new(channel::Number::Channel1),
            #[cfg(ledc_version = "1")]
            hs_channel2: channel::Channel::new(channel::Number::Channel2),
            #[cfg(ledc_version = "1")]
            hs_channel3: channel::Channel::new(channel::Number::Channel3),
            #[cfg(ledc_version = "1")]
            hs_channel4: channel::Channel::new(channel::Number::Channel4),
            #[cfg(ledc_version = "1")]
            hs_channel5: channel::Channel::new(channel::Number::Channel5),
            #[cfg(ledc_version = "1")]
            hs_channel6: channel::Channel::new(channel::Number::Channel6),
            #[cfg(ledc_version = "1")]
            hs_channel7: channel::Channel::new(channel::Number::Channel7),
        }
    }

    /// Set global slow clock source
    pub fn set_global_slow_clock(&mut self, clock_source: LSGlobalClkSource) {
        low_level::set_global_slow_clock(LEDC::regs(), clock_source);
    }
}
