//! Code for busy-waiting

use crate::{sysctl::Clocks, time::Hertz};
use cortex_m::peripheral::{syst::SystClkSource, SYST};
use embedded_hal::delay::DelayNs;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    sysclk: Hertz,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    #[inline]
    pub fn new(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay {
            syst,
            sysclk: clocks.sysclk,
        }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        if ns == 0 {
            return;
        }

        let mut rvr = ns.div_ceil(self.sysclk.to_ns());

        while rvr >= 1 << 24 {
            self.syst.set_reload((1 << 24) - 1);
            self.syst.clear_current();
            self.syst.enable_counter();
            while !self.syst.has_wrapped() {}
            self.syst.disable_counter();
            rvr -= 1 << 24;
        }

        assert!(rvr < (1 << 24));
        self.syst.set_reload(rvr);
        self.syst.clear_current();
        self.syst.enable_counter();
        while !self.syst.has_wrapped() {}
        self.syst.disable_counter();
    }
}
