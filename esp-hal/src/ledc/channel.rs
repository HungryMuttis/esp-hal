//! # LEDC channel
//!
//! ## Overview
//! The LEDC Channel module  provides a high-level interface to
//! configure and control individual PWM channels of the LEDC peripheral.
//!
//! ## Configuration
//! The module allows precise and flexible control over LED lighting and other
//! `Pulse-Width Modulation (PWM)` applications by offering configurable duty
//! cycles and frequencies.

use core::marker::PhantomData;

use super::{low_level, timer::TimerSpeed};
use crate::{
    gpio::{
        DriveMode,
        OutputConfig,
        interconnect::{self, PeripheralOutput},
    },
    ledc::timer::{Configured as ConfiguredTimer, Timer},
    peripherals::LEDC,
};

/// Fade parameter sub-errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FadeError {
    /// Duty change from start to end is out of range
    DutyRange,
    /// Duration too long for timer frequency and duty resolution
    Duration,
}

/// Channel errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid duty value
    Duty,
    /// Fade parameters invalid
    Fade(FadeError),
}

/// Channel number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Channel 0
    Channel0 = 0,
    /// Channel 1
    Channel1 = 1,
    /// Channel 2
    Channel2 = 2,
    /// Channel 3
    Channel3 = 3,
    /// Channel 4
    Channel4 = 4,
    /// Channel 5
    Channel5 = 5,
    #[cfg(ledc_channel_count = "8")]
    /// Channel 6
    Channel6 = 6,
    #[cfg(ledc_channel_count = "8")]
    /// Channel 7
    Channel7 = 7,
}

/// Channel configuration
#[derive(Copy, Clone, Debug, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config<'t, S: TimerSpeed> {
    /// A reference to the timer associated with this channel.
    pub timer: &'t Timer<S, ConfiguredTimer>,
    /// The duty cycle.
    pub duty: u32,
    /// The pin configuration (PushPull or OpenDrain).
    pub pin_config: DriveMode,
}

/// Typestate indicating an unconfigured channel.
#[derive(Clone, Copy, Debug)]
pub struct Unconfigured;

/// Typestate indicating a configured channel.
pub struct Configured {
    timer_duty: u8,
    timer_frequency: u32,
    pin: crate::gpio::PinGuard,
}

/// Channel struct
pub struct Channel<S: TimerSpeed, State = Unconfigured> {
    _phantom: PhantomData<S>,
    number: Number,
    state: State,
}

impl<S: TimerSpeed> Channel<S, Unconfigured> {
    /// Create a new instance of a channel
    pub(crate) fn new(number: Number) -> Self {
        Channel {
            _phantom: PhantomData,
            number,
            state: Unconfigured,
        }
    }

    /// Configure the channel, returning the configured channel.
    pub fn configure<'d>(
        self,
        config: Config<'_, S>,
        pin: impl PeripheralOutput<'d>,
    ) -> Result<Channel<S, Configured>, Error> {
        let timer_duty = config.timer.config().duty as u8;
        let timer_frequency = config.timer.config().frequency.as_hz();
        let timer_num = config.timer.number() as u8;

        let duty_range = 1u32 << timer_duty;
        let duty = config.duty.min(duty_range);

        let ledc = LEDC::regs();
        low_level::set_channel(ledc, self.number, timer_num, S::IS_HS);
        low_level::start_duty_without_fading(ledc, self.number, S::IS_HS);
        low_level::set_duty_hw(ledc, self.number, S::IS_HS, duty);
        low_level::update_channel(ledc, self.number, S::IS_HS);

        let output_signal = low_level::output_signal(self.number, S::IS_HS);

        let pin_out = pin.into();
        let out_cfg = OutputConfig::default().with_drive_mode(config.pin_config);
        pin_out.apply_output_config(&out_cfg);
        pin_out.set_output_enable(true);
        let pin_guard = pin_out.connect_with_guard(output_signal);

        Ok(Channel {
            _phantom: PhantomData,
            number: self.number,
            state: Configured {
                timer_duty,
                timer_frequency,
                pin: pin_guard,
            },
        })
    }
}

impl<S: TimerSpeed> Channel<S, Configured> {
    /// Set duty of channel
    pub fn set_duty(&self, duty: u32) -> Result<(), Error> {
        let duty_range = 1u32 << self.state.timer_duty;
        let duty_value = duty.min(duty_range);

        self.set_duty_hw(duty_value);

        Ok(())
    }

    /// Converts a percentage to a raw duty value based on current timer resolution
    pub fn percent_to_duty(&self, pct: u8) -> u32 {
        let pct = pct.min(100);
        let duty_range = (1u32 << self.state.timer_duty) - 1;
        (duty_range * pct as u32) / 100
    }

    /// Set duty % of channel
    pub fn set_duty_pct(&self, duty_pct: u8) -> Result<(), Error> {
        self.set_duty(self.percent_to_duty(duty_pct))
    }

    /// Start a duty fade from one raw duty value to another.
    ///
    /// There's a constraint on the combination of timer frequency, timer PWM
    /// duty resolution (the bit count), the fade "range" (abs(start-end)), and
    /// the duration:
    ///
    /// frequency * duration / ((1<<bit_count) * abs(start-end)) < 1024
    ///
    /// Small percentage changes, long durations, coarse PWM resolutions (that
    /// is, low bit counts), and high timer frequencies will all be more likely
    /// to fail this requirement.  If it does fail, this function will return
    /// an error Result.
    pub fn start_duty_fade(
        &self,
        start_duty: u32,
        end_duty: u32,
        duration_ms: u16,
    ) -> Result<(), Error> {
        let max_duty = (1u32 << self.state.timer_duty) - 1;
        let start_duty_value = start_duty.min(max_duty);
        let end_duty_value = end_duty.min(max_duty);

        let pwm_cycles = (duration_ms as u32) * self.state.timer_frequency / 1000;
        let abs_duty_diff = end_duty_value.abs_diff(start_duty_value);
        let duty_steps: u32 = u16::try_from(abs_duty_diff).unwrap_or(65535).into();

        let cycles_per_step: u16 = (pwm_cycles / duty_steps)
            .try_into()
            .map_err(|_| Error::Fade(FadeError::Duration))
            .and_then(|res| {
                if res > 1023 {
                    Err(Error::Fade(FadeError::Duration))
                } else {
                    Ok(res)
                }
            })?;

        let duty_per_cycle: u16 = (abs_duty_diff / duty_steps)
            .try_into()
            .map_err(|_| Error::Fade(FadeError::DutyRange))?;

        self.start_duty_fade_hw(
            start_duty_value,
            end_duty_value > start_duty_value,
            duty_steps as u16,
            cycles_per_step,
            duty_per_cycle,
        );

        Ok(())
    }

    /// Check whether a duty-cycle fade is running
    pub fn is_duty_fade_running(&self) -> bool {
        let ledc = LEDC::regs();
        low_level::is_duty_fade_running_hw(ledc, self.number, S::IS_HS)
    }

    /// Returns the unconfigured [`Channel`] and the consumed [`PeripheralOutput`]
    pub fn into_inner<'a>(self) -> (Channel<S, Unconfigured>, Option<crate::gpio::AnyPin<'a>>) {
        let ledc = LEDC::regs();
        low_level::set_duty_hw(ledc, self.number, S::IS_HS, 0);
        low_level::update_channel(ledc, self.number, S::IS_HS);

        let pin_num = self.state.pin.pin_number();

        drop(self.state.pin);
        let any_pin = pin_num.map(|n| unsafe { crate::gpio::AnyPin::steal(n) });

        (Channel::new(self.number), any_pin)
    }

    fn set_duty_hw(&self, duty: u32) {
        let ledc = LEDC::regs();
        low_level::set_duty_hw(ledc, self.number, S::IS_HS, duty);
        low_level::start_duty_without_fading(ledc, self.number, S::IS_HS);
        low_level::update_channel(ledc, self.number, S::IS_HS);
    }

    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        let ledc = LEDC::regs();
        low_level::start_duty_fade_hw(
            ledc,
            self.number,
            S::IS_HS,
            start_duty,
            duty_inc,
            duty_steps,
            cycles_per_step,
            duty_per_cycle,
        );
        low_level::update_channel(ledc, self.number, S::IS_HS);
    }
}

mod ehal1 {
    use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};

    use super::{Channel, Configured, Error};
    use crate::ledc::timer::TimerSpeed;

    impl embedded_hal::pwm::Error for Error {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl<S: TimerSpeed> ErrorType for Channel<S, Configured> {
        type Error = Error;
    }

    impl<S: TimerSpeed> SetDutyCycle for Channel<S, Configured> {
        fn max_duty_cycle(&self) -> u16 {
            (1 << self.state.timer_duty) - 1
        }

        fn set_duty_cycle(&mut self, mut duty: u16) -> Result<(), Self::Error> {
            let max = self.max_duty_cycle();
            duty = if duty > max { max } else { duty };
            self.set_duty_hw(duty as u32);
            Ok(())
        }
    }
}
