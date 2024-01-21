//! Code for GPIO pins

use core::marker::PhantomData;

/// All unlocked pin modes implement this
pub trait IsUnlocked {}

/// All input modes implement this
pub trait InputMode {}

/// All output modes implement this
pub trait OutputMode {}

/// OpenDrain modes implement this
pub trait OpenDrainMode {
    /// Is pull-up enabled
    fn pup() -> bool;
}

/// All the different Alternate Functions you can choose implement this
pub trait AlternateFunctionChoice {
    /// Which Alternate Function (numbered 1 through 15) is this?
    fn number() -> u32;
}

/// Input mode (type state)
pub struct Input<MODE>
where
    MODE: InputMode,
{
    _mode: PhantomData<MODE>,
}
impl<MODE> IsUnlocked for Input<MODE> where MODE: InputMode {}

/// Sub-mode of Input: Floating input (type state)
pub struct Floating;
impl InputMode for Floating {}
impl OpenDrainMode for Floating {
    /// Pull-up is not enabled
    #[inline]
    fn pup() -> bool {
        false
    }
}

/// Sub-mode of Input: Pulled down input (type state)
pub struct PullDown;
impl InputMode for PullDown {}

/// Sub-mode of Input: Pulled up input (type state)
pub struct PullUp;
impl InputMode for PullUp {}
impl OpenDrainMode for PullUp {
    /// Pull-up is enabled
    #[inline]
    fn pup() -> bool {
        true
    }
}

/// Tri-state
pub struct Tristate;
impl IsUnlocked for Tristate {}

/// Output mode (type state)
pub struct Output<MODE>
where
    MODE: OutputMode,
{
    _mode: PhantomData<MODE>,
}
impl<MODE> IsUnlocked for Output<MODE> where MODE: OutputMode {}

/// AlternateFunction mode (type state for a GPIO pin)
pub struct AlternateFunction<AF, MODE>
where
    AF: AlternateFunctionChoice,
    MODE: OutputMode,
{
    _func: PhantomData<AF>,
    _mode: PhantomData<MODE>,
}
impl<AF, MODE> IsUnlocked for AlternateFunction<AF, MODE>
where
    AF: AlternateFunctionChoice,
    MODE: OutputMode,
{
}

/// Sub-mode of Output/AlternateFunction: Push pull output (type state for
/// Output)
pub struct PushPull;
impl OutputMode for PushPull {}
impl OutputMode for PullDown {}
impl OutputMode for PullUp {}

/// Sub-mode of Output/AlternateFunction: Open drain output (type state for
/// Output)
pub struct OpenDrain<ODM>
where
    ODM: OpenDrainMode,
{
    _pull: PhantomData<ODM>,
}
impl<ODM> OutputMode for OpenDrain<ODM> where ODM: OpenDrainMode {}

/// Alternate function 1 (type state)
pub struct AF1;
impl AlternateFunctionChoice for AF1 {
    #[inline]
    fn number() -> u32 {
        1
    }
}

/// Alternate function 2 (type state)
pub struct AF2;
impl AlternateFunctionChoice for AF2 {
    #[inline]
    fn number() -> u32 {
        2
    }
}

/// Alternate function 3 (type state)
pub struct AF3;
impl AlternateFunctionChoice for AF3 {
    #[inline]
    fn number() -> u32 {
        3
    }
}

/// Alternate function 4 (type state)
pub struct AF4;
impl AlternateFunctionChoice for AF4 {
    #[inline]
    fn number() -> u32 {
        4
    }
}

/// Alternate function 5 (type state)
pub struct AF5;
impl AlternateFunctionChoice for AF5 {
    #[inline]
    fn number() -> u32 {
        5
    }
}

/// Alternate function 6 (type state)
pub struct AF6;
impl AlternateFunctionChoice for AF6 {
    #[inline]
    fn number() -> u32 {
        6
    }
}

/// Alternate function 7 (type state)
pub struct AF7;
impl AlternateFunctionChoice for AF7 {
    #[inline]
    fn number() -> u32 {
        7
    }
}

/// Alternate function 8 (type state)
pub struct AF8;
impl AlternateFunctionChoice for AF8 {
    #[inline]
    fn number() -> u32 {
        8
    }
}

/// Alternate function 9 (type state)
pub struct AF9;
impl AlternateFunctionChoice for AF9 {
    #[inline]
    fn number() -> u32 {
        9
    }
}

// 10 through 13 are not available on this chip.

/// Alternate function 14 (type state)
pub struct AF14;
impl AlternateFunctionChoice for AF14 {
    #[inline]
    fn number() -> u32 {
        14
    }
}

/// Pin is locked through the GPIOCR register
pub struct Locked;

/// Sets when a GPIO pin triggers an interrupt.
pub enum InterruptMode {
    /// Interrupt when level is low
    LevelLow,
    /// Interrupt when level is high
    LevelHigh,
    /// Interrupt on rising edge
    EdgeRising,
    /// Interrupt on falling edge
    EdgeFalling,
    /// Interrupt on both rising and falling edges
    EdgeBoth,
    /// Disable interrupts on this pin
    Disabled,
}

/// An internal macro to implement the GPIO functionality for each port
#[allow(clippy::crate_in_macro_def)] // We want to use the macro call's crate for `Sealed`
#[macro_export]
macro_rules! gpio_macro {
    ($chip_crate:ident, $GPIOX:ident, $gpiox:ident, $iopd:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use super::*;
            use $chip_crate::$GPIOX;

            /// Provides mutual-exclusion for certain GPIO operations (such as
            /// selecting an alternate mode) that can't be done atomically.
            pub struct GpioControl {
                _0: (),
            }

            /// GPIO parts
            pub struct Parts {
                /// Pass an &mut reference to methods that require it.
                pub control: GpioControl,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                /// Break this GPIO port into separate pins
                #[inline]
                fn split(self, pc: &sysctl::PowerControl) -> Parts {
                    sysctl::control_power(
                        pc, sysctl::Domain::$iopd,
                        sysctl::RunMode::Run, sysctl::PowerState::On);
                    sysctl::reset(pc, sysctl::Domain::$iopd);

                    Parts {
                        control: GpioControl { _0: () },
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> embedded_hal::digital::ErrorType for $PXx<Output<MODE>> where MODE: OutputMode {
                type Error = ::core::convert::Infallible;
            }

            impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXx<Output<MODE>> where MODE: OutputMode {
                #[inline]
                fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                    let p = unsafe { &*$GPIOX::ptr() };
                    Ok(bb::read_bit(&p.data, self.i))
                }

                #[inline]
                fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                    self.is_set_high().map(|b| !b)
                }
            }

            impl<MODE> embedded_hal::digital::OutputPin for $PXx<Output<MODE>> where MODE: OutputMode {
                #[inline]
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    let p = unsafe { &*$GPIOX::ptr() };
                    unsafe { bb::change_bit(&p.data, self.i, true); }
                    Ok(())
                }

                #[inline]
                fn set_low(&mut self) -> Result<(), Self::Error> {
                    let p = unsafe { &*$GPIOX::ptr() };
                    unsafe { bb::change_bit(&p.data, self.i, false); }
                    Ok(())
                }
            }

            impl<MODE> embedded_hal::digital::ErrorType for $PXx<Input<MODE>> where MODE: InputMode {
                type Error = ::core::convert::Infallible;
            }

            impl<MODE> embedded_hal::digital::InputPin for $PXx<Input<MODE>> where MODE: InputMode {
                #[inline]
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    let p = unsafe { &*$GPIOX::ptr() };
                    Ok(bb::read_bit(&p.data, self.i))
                }

                #[inline]
                fn is_low(&mut self) -> Result<bool, Self::Error> {
                    self.is_high().map(|b| !b)
                }
            }

            impl<MODE> $PXx<Input<MODE>> where MODE: InputMode {
                /// Enables or disables interrupts on this GPIO pin.
                pub fn set_interrupt_mode(&mut self, mode: InterruptMode) {
                    let p = unsafe { &*$GPIOX::ptr() };
                    unsafe { bb::change_bit(&p.im, self.i, false); }
                    match mode {
                        InterruptMode::LevelHigh => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                            // IS |= self.i;
                            unsafe { bb::change_bit(&p.is, self.i, true); }
                            // IBE &= ~self.i;
                            unsafe { bb::change_bit(&p.ibe, self.i, false); }
                            // IEV |= self.i;
                            unsafe { bb::change_bit(&p.iev, self.i, true); }
                            // IM |= self.i;
                            unsafe { bb::change_bit(&p.im, self.i, true); }
                        },
                        InterruptMode::LevelLow => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                            // IS |= self.i;
                            unsafe { bb::change_bit(&p.is, self.i, true); }
                            // IBE &= ~self.i;
                            unsafe { bb::change_bit(&p.ibe, self.i, false); }
                            // IEV &= ~self.i;
                            unsafe { bb::change_bit(&p.iev, self.i, false); }
                            // IM |= self.i;
                            unsafe { bb::change_bit(&p.im, self.i, true); }
                        },
                        InterruptMode::EdgeRising => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                            // IS &= ~self.i;
                            unsafe { bb::change_bit(&p.is, self.i, false); }
                            // IBE &= ~self.i;
                            unsafe { bb::change_bit(&p.ibe, self.i, false); }
                            // IEV |= self.i;
                            unsafe { bb::change_bit(&p.iev, self.i, true); }
                            // IM |= self.i;
                            unsafe { bb::change_bit(&p.im, self.i, true); }
                        },
                        InterruptMode::EdgeFalling => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                            // IS &= ~self.i;
                            unsafe { bb::change_bit(&p.is, self.i, false); }
                            // IBE &= ~self.i;
                            unsafe { bb::change_bit(&p.ibe, self.i, false); }
                            // IEV &= ~self.i;
                            unsafe { bb::change_bit(&p.iev, self.i, false); }
                            // IM |= self.i;
                            unsafe { bb::change_bit(&p.im, self.i, true); }
                        },
                        InterruptMode::EdgeBoth => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                            // IS &= ~self.i;
                            unsafe { bb::change_bit(&p.is, self.i, false); }
                            // IBE |= self.i;
                            unsafe { bb::change_bit(&p.ibe, self.i, true); }
                            // IEV |= self.i;
                            unsafe { bb::change_bit(&p.iev, self.i, true); }
                            // IM |= self.i;
                            unsafe { bb::change_bit(&p.im, self.i, true); }
                        },
                        InterruptMode::Disabled => {
                            // IM &= ~self.i;
                            unsafe { bb::change_bit(&p.im, self.i, false); }
                        },
                    }
                }

                /// Returns the current interrupt status for this pin.
                #[inline]
                pub fn get_interrupt_status(&self) -> bool {
                    let p = unsafe { &*$GPIOX::ptr() };
                    bb::read_bit(&p.mis, self.i)
                }

                /// Marks the interrupt for this pin as handled. You should
                /// call this (or perform its functionality) from the ISR.
                #[inline]
                pub fn clear_interrupt(&self) {
                    let p = unsafe { &*$GPIOX::ptr() };
                    unsafe { bb::change_bit(&p.icr, self.i, true); }
                }
            }

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> crate::Sealed for $PXi<MODE> {}

                impl<MODE> $PXi<MODE> where MODE: IsUnlocked {
                    /// Configures the pin to serve as alternate function 1 through 15.
                    /// Disables open-drain to make the output a push-pull.
                    pub fn into_af_push_pull<AF>(
                        self,
                        _gpio_control: &mut GpioControl,
                    ) -> $PXi<AlternateFunction<AF, PushPull>> where AF: AlternateFunctionChoice {
                        let p = unsafe { &*$GPIOX::ptr() };
                        let mask = 0xF << ($i * 4);
                        let bits = AF::number() << ($i * 4);
                        unsafe {
                            p.pctl.modify(|r, w| w.bits((r.bits() & !mask) | bits));
                        }
                        unsafe { bb::change_bit(&p.afsel, $i, true); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to serve as alternate function 1 through 15 with
                    /// a weak pull-up resistor.
                    pub fn into_af_pull_up<AF>(
                        self,
                        _gpio_control: &mut GpioControl,
                    ) -> $PXi<AlternateFunction<AF, PullUp>> where AF: AlternateFunctionChoice {
                        let p = unsafe { &*$GPIOX::ptr() };
                        let mask = 0xF << ($i * 4);
                        let bits = AF::number() << ($i * 4);
                        unsafe {
                            p.pctl.modify(|r, w| w.bits((r.bits() & !mask) | bits));
                        }
                        unsafe { bb::change_bit(&p.afsel, $i, true); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, true); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to serve as alternate function 1 through 15 with
                    /// a weak pull-down resistor.
                    pub fn into_af_pull_down<AF>(
                        self,
                        _gpio_control: &mut GpioControl,
                    ) -> $PXi<AlternateFunction<AF, PullDown>> where AF: AlternateFunctionChoice {
                        let p = unsafe { &*$GPIOX::ptr() };
                        let mask = 0xF << ($i * 4);
                        let bits = AF::number() << ($i * 4);
                        unsafe {
                            p.pctl.modify(|r, w| w.bits((r.bits() & !mask) | bits));
                        }
                        unsafe { bb::change_bit(&p.afsel, $i, true); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, true); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to serve as alternate function 1 through 15.
                    /// Enables open-drain (useful for I2C SDA, for example).
                    pub fn into_af_open_drain<AF, ODM>(
                        self,
                        _gpio_control: &mut GpioControl,
                    ) -> $PXi<AlternateFunction<AF, OpenDrain<ODM>>> where AF: AlternateFunctionChoice, ODM: OpenDrainMode {
                        let p = unsafe { &*$GPIOX::ptr() };
                        let mask = 0xF << ($i * 4);
                        let bits = AF::number() << ($i * 4);
                        unsafe {
                            p.pctl.modify(|r, w| w.bits((r.bits() & !mask) | bits));
                        }
                        unsafe { bb::change_bit(&p.afsel, $i, true); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, true); }
                        unsafe { bb::change_bit(&p.pur, $i, ODM::pup()); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(
                        self
                    ) -> $PXi<Input<Floating>> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(
                        self
                    ) -> $PXi<Input<PullDown>> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, true); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(
                        self
                    ) -> $PXi<Input<PullUp>> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, true); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open drain output pin
                    pub fn into_open_drain_output<ODM>(
                        self
                    ) -> $PXi<Output<OpenDrain<ODM>>> where ODM: OpenDrainMode {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, true); }
                        unsafe { bb::change_bit(&p.odr, $i, true); }
                        unsafe { bb::change_bit(&p.pur, $i, ODM::pup()); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push pull output pin
                    pub fn into_push_pull_output(
                        self
                    ) -> $PXi<Output<PushPull>> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, true); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        unsafe { bb::change_bit(&p.den, $i, true); }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as tri-state
                    pub fn into_tri_state(
                        self
                    ) -> $PXi<Tristate> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.den, $i, false); }
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        $PXi { _mode: PhantomData }
                    }

                }

                impl<MODE> $PXi<MODE> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    #[inline]
                    pub fn downgrade(self) -> $PXx<MODE> {
                        $PXx {
                            i: $i,
                            _mode: self._mode,
                        }
                    }
                }

                impl<MODE> embedded_hal::digital::ErrorType for $PXi<Output<MODE>> where MODE: OutputMode {
                    type Error = ::core::convert::Infallible;
                }

                impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXi<Output<MODE>> where MODE: OutputMode {
                    #[inline]
                    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        Ok(bb::read_bit(&p.data, $i))
                    }

                    #[inline]
                    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                        self.is_set_high().map(|b| !b)
                    }
                }

                impl<MODE> embedded_hal::digital::OutputPin for $PXi<Output<MODE>> where MODE: OutputMode {
                    #[inline]
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.data, $i, true); }
                        Ok(())
                    }

                    #[inline]
                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.data, $i, false); }
                        Ok(())
                    }
                }

                impl<MODE> embedded_hal::digital::ErrorType for $PXi<Input<MODE>> where MODE: InputMode {
                    type Error = ::core::convert::Infallible;
                }

                impl<MODE> embedded_hal::digital::InputPin for $PXi<Input<MODE>> where MODE: InputMode {
                    #[inline]
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        Ok(bb::read_bit(&p.data, $i))
                    }

                    #[inline]
                    fn is_low(&mut self) -> Result<bool, Self::Error> {
                        self.is_high().map(|b| !b)
                    }
                }

                impl<MODE> $PXi<Input<MODE>> where MODE: InputMode {
                    /// Enables or disables interrupts on this GPIO pin.
                    pub fn set_interrupt_mode(&mut self, mode: InterruptMode) {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.im, $i, false); }
                        match mode {
                            InterruptMode::LevelHigh => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                                // IS |= $i;
                                unsafe { bb::change_bit(&p.is, $i, true); }
                                // IBE &= ~$i;
                                unsafe { bb::change_bit(&p.ibe, $i, false); }
                                // IEV |= $i;
                                unsafe { bb::change_bit(&p.iev, $i, true); }
                                // IM |= $i;
                                unsafe { bb::change_bit(&p.im, $i, true); }
                            },
                            InterruptMode::LevelLow => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                                // IS |= $i;
                                unsafe { bb::change_bit(&p.is, $i, true); }
                                // IBE &= ~$i;
                                unsafe { bb::change_bit(&p.ibe, $i, false); }
                                // IEV &= ~$i;
                                unsafe { bb::change_bit(&p.iev, $i, false); }
                                // IM |= $i;
                                unsafe { bb::change_bit(&p.im, $i, true); }
                            },
                            InterruptMode::EdgeRising => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                                // IS &= ~$i;
                                unsafe { bb::change_bit(&p.is, $i, false); }
                                // IBE &= ~$i;
                                unsafe { bb::change_bit(&p.ibe, $i, false); }
                                // IEV |= $i;
                                unsafe { bb::change_bit(&p.iev, $i, true); }
                                // IM |= $i;
                                unsafe { bb::change_bit(&p.im, $i, true); }
                            },
                            InterruptMode::EdgeFalling => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                                // IS &= ~$i;
                                unsafe { bb::change_bit(&p.is, $i, false); }
                                // IBE &= ~$i;
                                unsafe { bb::change_bit(&p.ibe, $i, false); }
                                // IEV &= ~$i;
                                unsafe { bb::change_bit(&p.iev, $i, false); }
                                // IM |= $i;
                                unsafe { bb::change_bit(&p.im, $i, true); }
                            },
                            InterruptMode::EdgeBoth => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                                // IS &= ~$i;
                                unsafe { bb::change_bit(&p.is, $i, false); }
                                // IBE |= $i;
                                unsafe { bb::change_bit(&p.ibe, $i, true); }
                                // IEV |= $i;
                                unsafe { bb::change_bit(&p.iev, $i, true); }
                                // IM |= $i;
                                unsafe { bb::change_bit(&p.im, $i, true); }
                            },
                            InterruptMode::Disabled => {
                                // IM &= ~$i;
                                unsafe { bb::change_bit(&p.im, $i, false); }
                            },
                        }
                    }

                    /// Returns the current interrupt status for this pin.
                    #[inline]
                    pub fn get_interrupt_status(&self) -> bool {
                        let p = unsafe { &*$GPIOX::ptr() };
                        bb::read_bit(&p.mis, $i)
                    }

                    /// Marks the interrupt for this pin as handled. You should
                    /// call this (or perform its functionality) from the ISR.
                    #[inline]
                    pub fn clear_interrupt(&self) {
                        let p = unsafe { &*$GPIOX::ptr() };
                        unsafe { bb::change_bit(&p.icr, $i, true); }
                    }
                }

                impl $PXi<Locked> {
                    /// Unlock a GPIO so that it can be used. This is required
                    /// on 'special' GPIOs that the manufacturer doesn't want
                    /// you to change by accident - like NMI and JTAG pins.
                    pub fn unlock(self, _gpio_control: &mut GpioControl) -> $PXi<Tristate> {
                        let p = unsafe { &*$GPIOX::ptr() };
                        p.lock.write(|w| w.lock().key());
                        p.cr.modify(|_, w| unsafe { w.bits(1 << $i) });
                        p.lock.write(|w| w.lock().unlocked());
                        unsafe { bb::change_bit(&p.den, $i, false); }
                        unsafe { bb::change_bit(&p.afsel, $i, false); }
                        unsafe { bb::change_bit(&p.dir, $i, false); }
                        unsafe { bb::change_bit(&p.odr, $i, false); }
                        unsafe { bb::change_bit(&p.pur, $i, false); }
                        unsafe { bb::change_bit(&p.pdr, $i, false); }
                        $PXi { _mode: PhantomData }
                    }
                }
            )+
        }
    }
}

// End of file
