//! Time units

use cortex_m::peripheral::DWT;

/// Bits per second
#[derive(Clone, Copy)]
pub struct Bps(pub u32);

/// Hertz
#[derive(Clone, Copy)]
pub struct Hertz(pub u32);

impl Hertz {
    /// Compute the number of nanoseconds (round up) per cycle
    ///
    /// Example: 80MHz will return 13
    #[inline]
    pub fn to_ns(self) -> u32 {
        const ONE_GIGAHERTZ: u32 = 1_000_000_000;
        if self.0 == 0 {
            panic!("hertz cannot be zero");
        } else {
            // This division will never be 0 provided that hertz is < 1GHz which seems unlikely
            debug_assert!(self.0 <= ONE_GIGAHERTZ);
            ONE_GIGAHERTZ.div_ceil(self.0)
        }
    }
}

/// KiloHertz
#[derive(Clone, Copy)]
pub struct KiloHertz(pub u32);

/// MegaHertz
#[derive(Clone, Copy)]
pub struct MegaHertz(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }
}

impl From<KiloHertz> for Hertz {
    #[inline]
    fn from(val: KiloHertz) -> Self {
        Hertz(val.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    #[inline]
    fn from(val: MegaHertz) -> Self {
        Hertz(val.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    #[inline]
    fn from(val: MegaHertz) -> Self {
        KiloHertz(val.0 * 1_000)
    }
}

/// A monotonic nondecreasing timer
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Returns the frequency at which the monotonic timer is operating at
    #[inline]
    pub fn frequency(self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    #[inline]
    pub fn now(self) -> Instant {
        Instant {
            now: DWT::get_cycle_count(),
        }
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    #[inline]
    pub fn elapsed(self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}

// Copied from embedded-hal@0.2.7
/// A count down timer
///
/// # Contract
///
/// - `self.start(count); block!(self.wait());` MUST block for AT LEAST the time specified by
/// `count`.
///
/// *Note* that the implementer doesn't necessarily have to be a *downcounting* timer; it could also
/// be an *upcounting* timer as long as the above contract is upheld.
///
/// # Examples
///
/// You can use this timer to create delays
///
/// ```
/// extern crate embedded_hal as hal;
/// #[macro_use(block)]
/// extern crate nb;
///
/// use hal::prelude::*;
///
/// fn main() {
///     let mut led: Led = {
///         // ..
/// #       Led
///     };
///     let mut timer: Timer6 = {
///         // ..
/// #       Timer6
///     };
///
///     Led.on();
///     timer.start(1.s());
///     block!(timer.wait()); // blocks for 1 second
///     Led.off();
/// }
///
/// # extern crate void;
/// # use void::Void;
/// # struct Seconds(u32);
/// # trait U32Ext { fn s(self) -> Seconds; }
/// # impl U32Ext for u32 { fn s(self) -> Seconds { Seconds(self) } }
/// # struct Led;
/// # impl Led {
/// #     pub fn off(&mut self) {}
/// #     pub fn on(&mut self) {}
/// # }
/// # struct Timer6;
/// # impl hal::timer::CountDown for Timer6 {
/// #     type Time = Seconds;
/// #     fn start<T>(&mut self, _: T) where T: Into<Seconds> {}
/// #     fn wait(&mut self) -> ::nb::Result<(), Void> { Ok(()) }
/// # }
/// ```
pub trait CountDown {
    /// The unit of time used by this timer
    type Time;

    /// Starts a new count down
    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>;

    /// Non-blockingly "waits" until the count down finishes
    ///
    /// # Contract
    ///
    /// - If `Self: Periodic`, the timer will start a new count down right after the last one
    /// finishes.
    /// - Otherwise the behavior of calling `wait` after the last call returned `Ok` is UNSPECIFIED.
    /// Implementers are suggested to panic on this scenario to signal a programmer error.
    fn wait(&mut self) -> nb::Result<(), void::Void>;
}

/// Marker trait that indicates that a timer is periodic
pub trait Periodic {}
