//! # LEDC timer
//!
//! ## Overview
//! The LEDC Timer provides a high-level interface to configure and control
//! individual timers of the `LEDC` peripheral.
//!
//! ## Configuration
//! The module allows precise and flexible control over timer configurations,
//! duty cycles and frequencies, making it ideal for Pulse-Width Modulation
//! (PWM) applications and LED lighting control.
//!
//! LEDC uses APB as clock source.

use core::marker::PhantomData;

#[cfg(ledc_version = "1")]
use super::HighSpeed;
use super::{LowSpeed, Speed, low_level};
use crate::{peripherals::LEDC, time::Rate};

const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;

/// Timer errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid Divisor
    Divisor,
    /// Frequency unset
    FrequencyUnset,
}

/// Clock source for LS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// APB clock.
    APBClk,
    // TODO SLOWClk
}

/// Timer number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Timer 0.
    Timer0 = 0,
    /// Timer 1.
    Timer1 = 1,
    /// Timer 2.
    Timer2 = 2,
    /// Timer 3.
    Timer3 = 3,
}

/// Number of bits reserved for duty cycle adjustment
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duty {
    /// 1-bit resolution for duty cycle adjustment.
    Bit1 = 1,
    /// 2-bit resolution for duty cycle adjustment.
    Bit2,
    /// 3-bit resolution for duty cycle adjustment.
    Bit3,
    /// 4-bit resolution for duty cycle adjustment.
    Bit4,
    /// 5-bit resolution for duty cycle adjustment.
    Bit5,
    /// 6-bit resolution for duty cycle adjustment.
    Bit6,
    /// 7-bit resolution for duty cycle adjustment.
    Bit7,
    /// 8-bit resolution for duty cycle adjustment.
    Bit8,
    /// 9-bit resolution for duty cycle adjustment.
    Bit9,
    /// 10-bit resolution for duty cycle adjustment.
    Bit10,
    /// 11-bit resolution for duty cycle adjustment.
    Bit11,
    /// 12-bit resolution for duty cycle adjustment.
    Bit12,
    /// 13-bit resolution for duty cycle adjustment.
    Bit13,
    /// 14-bit resolution for duty cycle adjustment.
    Bit14,
    #[cfg(ledc_version = "1")]
    /// 15-bit resolution for duty cycle adjustment.
    Bit15,
    #[cfg(ledc_version = "1")]
    /// 16-bit resolution for duty cycle adjustment.
    Bit16,
    #[cfg(ledc_version = "1")]
    /// 17-bit resolution for duty cycle adjustment.
    Bit17,
    #[cfg(ledc_version = "1")]
    /// 18-bit resolution for duty cycle adjustment.
    Bit18,
    #[cfg(ledc_version = "1")]
    /// 19-bit resolution for duty cycle adjustment.
    Bit19,
    #[cfg(ledc_version = "1")]
    /// 20-bit resolution for duty cycle adjustment.
    Bit20,
}

impl TryFrom<u32> for Duty {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            1 => Self::Bit1,
            2 => Self::Bit2,
            3 => Self::Bit3,
            4 => Self::Bit4,
            5 => Self::Bit5,
            6 => Self::Bit6,
            7 => Self::Bit7,
            8 => Self::Bit8,
            9 => Self::Bit9,
            10 => Self::Bit10,
            11 => Self::Bit11,
            12 => Self::Bit12,
            13 => Self::Bit13,
            14 => Self::Bit14,
            #[cfg(ledc_version = "1")]
            15 => Self::Bit15,
            #[cfg(ledc_version = "1")]
            16 => Self::Bit16,
            #[cfg(ledc_version = "1")]
            17 => Self::Bit17,
            #[cfg(ledc_version = "1")]
            18 => Self::Bit18,
            #[cfg(ledc_version = "1")]
            19 => Self::Bit19,
            #[cfg(ledc_version = "1")]
            20 => Self::Bit20,
            _ => Err(())?,
        })
    }
}

/// Timer configuration
#[derive(Copy, Clone, Debug, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The duty cycle resolution.
    pub duty: Duty,
    /// The clock source for the timer.
    pub clock_source: ClockSource,
    /// The frequency of the PWM signal in Hertz.
    pub frequency: Rate,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            duty: Duty::Bit8,
            clock_source: ClockSource::APBClk,
            frequency: Rate::from_khz(1),
        }
    }
}

/// Trait defining the type of timer source
pub trait TimerSpeed: Speed {
    #[doc(hidden)]
    fn apply_config(num: Number, config: Config) -> Result<bool, Error>;
}

/// Timer source type for LowSpeed timers
impl TimerSpeed for LowSpeed {
    fn apply_config(number: Number, config: Config) -> Result<bool, Error> {
        let src_freq: u32 = low_level::ls_freq_hw(config.clock_source).as_hz();
        let precision = 1 << config.duty as u32;
        let frequency: u32 = config.frequency.as_hz();

        #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused_mut))]
        let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

        #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused_mut))]
        let mut use_ref_tick = false;

        #[cfg(soc_has_clock_node_ref_tick)]
        if divisor > LEDC_TIMER_DIV_NUM_MAX {
            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
            // source.
            use_ref_tick = true;
            divisor = (1_000_000u64 << 8) / frequency as u64 / precision as u64;
        }

        if !(256..=LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
            return Err(Error::Divisor);
        }

        let ledc = LEDC::regs();
        low_level::ls_configure_hw(
            ledc,
            number,
            divisor as u32,
            config.duty as u8,
            use_ref_tick,
        );
        low_level::ls_update_hw(ledc, number);

        Ok(use_ref_tick)
    }
}

#[cfg(ledc_version = "1")]
/// Timer source type for HighSpeed timers
impl TimerSpeed for HighSpeed {
    fn apply_config(number: Number, config: Config) -> Result<bool, Error> {
        let src_freq: u32 = low_level::hs_freq_hw(config.clock_source).as_hz();
        let precision = 1 << config.duty as u32;
        let frequency: u32 = config.frequency.as_hz();

        let divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

        if !(256..=LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
            return Err(Error::Divisor);
        }

        let ledc = LEDC::regs();
        low_level::hs_configure_hw(
            ledc,
            number,
            divisor as u32,
            config.duty as u8,
            config.clock_source,
        );
        low_level::hs_update_hw();

        Ok(false)
    }
}

/// Typestate indicating an unconfigured timer.
#[derive(Clone, Copy, Debug)]
pub struct Unconfigured;

/// Typestate indicating a configured timer.
#[derive(Clone, Copy, Debug)]
pub struct Configured {
    pub(crate) config: Config,
    #[cfg(soc_has_clock_node_ref_tick)]
    pub(crate) use_ref_tick: bool,
}

/// Timer struct
#[derive(Debug)]
pub struct Timer<S: TimerSpeed, State = Unconfigured> {
    _phantom: PhantomData<S>,
    number: Number,
    state: State,
}

impl<S: TimerSpeed> Timer<S, Unconfigured> {
    /// Create a new instance of a timer
    pub(crate) fn new(number: Number) -> Self {
        Self {
            _phantom: PhantomData,
            number,
            state: Unconfigured,
        }
    }

    /// Configure the timer
    pub fn configure(self, config: Config) -> Result<Timer<S, Configured>, Error> {
        #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused))]
        let use_ref_tick = S::apply_config(self.number, config)?;

        Ok(Timer {
            _phantom: PhantomData,
            number: self.number,
            state: Configured {
                config,
                #[cfg(soc_has_clock_node_ref_tick)]
                use_ref_tick,
            },
        })
    }
}

impl<S: TimerSpeed> Timer<S, Configured> {
    /// Returns the current configuration of the timer
    pub fn config(&self) -> Config {
        self.state.config
    }

    /// Returns the timer number
    pub fn number(&self) -> Number {
        self.number
    }

    /// Reconfigures the timer with new settings
    pub fn reconfigure(&mut self, config: Config) -> Result<(), Error> {
        #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused))]
        let use_ref_tick = S::apply_config(self.number, config)?;

        self.state.config = config;
        #[cfg(soc_has_clock_node_ref_tick)]
        {
            self.state.use_ref_tick = use_ref_tick;
        }

        Ok(())
    }
}
