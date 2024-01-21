//! Serial code that is generic to both the TM4C123 and TM4C129, such as the pin traits.

// The macro is required for the "sealed trait" pattern to work:
// the traits and the gpios have to be defined in the same crate

/// An internal macro to generate the UART traits
#[allow(clippy::crate_in_macro_def)] // We want to use the macro call's crate for `Sealed`
#[macro_export]
macro_rules! uart_traits_macro {
    () => {
        /// TX pin
        pub trait TxPin<UART>: crate::Sealed {}

        /// RX pin
        pub trait RxPin<UART>: crate::Sealed {}

        /// CTS pin
        pub trait CtsPin<UART>: crate::Sealed {
            /// Enables the CTS functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        /// DCD pin
        pub trait DcdPin<UART>: crate::Sealed {
            /// Enables the DCD functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        /// DSR pin
        pub trait DsrPin<UART>: crate::Sealed {
            /// Enables the DSR functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        /// DTR pin
        pub trait DtrPin<UART>: crate::Sealed {
            /// Enables the DTR functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        /// RI pin
        pub trait RiPin<UART>: crate::Sealed {
            /// Enables the RI functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        /// RTS pin
        pub trait RtsPin<UART>: crate::Sealed {
            /// Enables the RTS functionality if a valid pin is given (not `()`).
            fn enable(&mut self, _uart: &mut UART);
        }

        impl<U> TxPin<U> for () {}

        impl<U> RxPin<U> for () {}

        impl<U> CtsPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
        impl<U> DcdPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
        impl<U> DsrPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
        impl<U> DtrPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
        impl<U> RiPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
        impl<U> RtsPin<U> for () {
            #[inline]
            fn enable(&mut self, _uart: &mut U) {
                // Do nothing
            }
        }
    };
}

/// An internal macro to implement the UART functionality for each peripheral
#[macro_export]
macro_rules! uart_hal_macro {
    ($(
        $UARTX:ident: ($powerDomain:ident, $uartX:ident),
    )+) => {
        $crate::uart_traits_macro!();

        $(
            impl<TX, RX, RTS, CTS> Serial<$UARTX, TX, RX, RTS, CTS> {
                /// Configures a UART peripheral to provide serial communication
                pub fn $uartX(
                    mut uart: $UARTX,
                    tx_pin: TX,
                    rx_pin: RX,
                    mut rts_pin: RTS,
                    mut cts_pin: CTS,
                    baud_rate: Bps,
                    clocks: &Clocks,
                    pc: &sysctl::PowerControl
                ) -> Self
                where
                    TX: TxPin<$UARTX>,
                    RX: RxPin<$UARTX>,
                    CTS: CtsPin<$UARTX>,
                    RTS: RtsPin<$UARTX>,
                {
                    // Enable UART peripheral clocks
                    sysctl::control_power(
                        pc, sysctl::Domain::$powerDomain,
                        sysctl::RunMode::Run, sysctl::PowerState::On);
                    sysctl::reset(pc, sysctl::Domain::$powerDomain);

                    // Reset UART
                    uart.ctl.reset();

                    // Calculate baud rate dividers
                    // baud_int = 64 * (sys_clk / (16 * baud))
                    // baud_int = 4 * (sys_clk / baud)
                    // baud_int = ((8 * sys_clk) / baud) / 2, plus + 1 to round correctly
                    let baud_int: u32 = (((clocks.sysclk.0 * 8) / baud_rate.0) + 1) / 2;

                    // Set baud rate
                    uart.ibrd.write(|w|
                        unsafe { w.divint().bits((baud_int / 64) as u16) });
                    uart.fbrd.write(|w|
                        unsafe { w.divfrac().bits((baud_int % 64) as u8) });

                    // Set data bits / parity / stop bits / enable fifo
                    uart.lcrh.write(|w| w.wlen()._8().fen().bit(true));

                    // Activate flow control (if desired)
                    rts_pin.enable(&mut uart);
                    cts_pin.enable(&mut uart);

                    // Enable uart
                    uart.ctl.modify(|_, w| w.rxe().bit(true).txe().bit(true).uarten().bit(true));

                    Serial { uart, tx_pin, rx_pin, rts_pin, cts_pin }
                }

                /// Change the current baud rate for the UART. We need the
                /// `clocks` object in order to calculate the magic baud rate
                /// register values.
                pub fn change_baud_rate(&mut self, baud_rate: Bps, clocks: &Clocks) {
                    // Stop UART
                    self.uart.ctl.modify(|_, w| w.uarten().bit(false));

                    // Calculate baud rate dividers
                    let baud_int: u32 = (((clocks.sysclk.0 * 8) / baud_rate.0) + 1) / 2;

                    // Set baud rate
                    self.uart.ibrd.write(|w|
                        unsafe { w.divint().bits((baud_int / 64) as u16) });
                    self.uart.fbrd.write(|w|
                        unsafe { w.divfrac().bits((baud_int % 64) as u8) });

                    // Set data bits / parity / stop bits / enable fifo
                    // If you don't write to this register, the baud rate change doesn't take effect
                    self.uart.lcrh.write(|w| w.wlen()._8().fen().bit(true));

                    // Start UART again
                    self.uart.ctl.modify(|_, w| w.uarten().bit(true));
                }

                /// Splits the `Serial` abstraction into a transmitter and a
                /// receiver half. If you do this you can transmit and receive
                /// in different threads.
                #[inline]
                pub fn split(self) -> (Tx<$UARTX, TX, RTS>, Rx<$UARTX, RX, CTS>) {
                    (
                        Tx {
                            uart: self.uart,
                            pin: self.tx_pin,
                            flow_pin: self.rts_pin,
                        },
                        Rx {
                            _uart: PhantomData,
                            pin: self.rx_pin,
                            flow_pin: self.cts_pin,
                        },
                    )
                }

                // /// Write a complete string to the UART.
                // pub fn write_all<I: ?Sized>(&mut self, data: &I)
                // where
                //     I: AsRef<[u8]>,
                // {
                //     for octet in data.as_ref().iter() {
                //         block!(self.write(*octet)).unwrap(); // E = Void
                //     }
                // }

                /// Re-combine a split UART
                #[inline]
                pub fn combine(tx: Tx<$UARTX, TX, RTS>, rx: Rx<$UARTX, RX, CTS>) -> Serial<$UARTX, TX, RX, RTS, CTS> {
                    Serial {
                        uart: tx.uart,
                        rx_pin: rx.pin,
                        tx_pin: tx.pin,
                        rts_pin: tx.flow_pin,
                        cts_pin: rx.flow_pin,
                    }
                }

                /// Releases the UART peripheral and associated pins
                pub fn free(self) -> ($UARTX, TX, RX, RTS, CTS) {
                    (self.uart, self.tx_pin, self.rx_pin, self.rts_pin, self.cts_pin)
                }
            }

            impl<TX, RX, RTS, CTS> embedded_io::ErrorType for Serial<$UARTX, TX, RX, RTS, CTS> {
                type Error = Error;
            }

            impl<TX, RX, RTS, CTS> embedded_io::WriteReady for Serial<$UARTX, TX, RX, RTS, CTS> {
                #[inline]
                fn write_ready(&mut self) -> Result<bool, Self::Error> {
                    Ok(self.uart.fr.read().txff().bit_is_clear()) // 0 means not full, which means ready to write
                }
            }

            impl<TX, RX, RTS, CTS> embedded_io::Write for Serial<$UARTX, TX, RX, RTS, CTS> {
                fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                    // Get first and remaining bytes
                    let Some((first, rem)) = buf.split_first() else {
                        return Ok(0); // Only allowed because buf is empty
                    };

                    // We must transmit at least one byte, blocking if needed
                    while self.uart.fr.read().txff().bit_is_set() {}
                    // Write the first byte
                    self.uart.dr.write(|w| unsafe { w.data().bits(*first) });

                    let mut num_sent = 1;
                    // Try to write the remaining bytes
                    for byte in rem {
                        // If the transmit fifo is full, return immedietly
                        if self.uart.fr.read().txff().bit_is_set() {
                            return Ok(num_sent);
                        }

                        // Write this byte
                        self.uart.dr.write(|w| unsafe { w.data().bits(*byte) });
                        num_sent += 1;
                    }
                    Ok(num_sent)
                }

                #[inline]
                fn flush(&mut self) -> Result<(), Self::Error> {
                    while self.uart.fr.read().busy().bit_is_set() {}
                    Ok(())
                }
            }

            impl<TX, RX, RTS, CTS> embedded_io::ReadReady for Serial<$UARTX, TX, RX, RTS, CTS> {
                #[inline]
                fn read_ready(&mut self) -> Result<bool, Self::Error> {
                    Ok(self.uart.fr.read().rxfe().bit_is_clear()) // 0 means not empty, which means ready to read
                }
            }

            impl<TX, RX, RTS, CTS> embedded_io::Read for Serial<$UARTX, TX, RX, RTS, CTS> {
                fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                    // Get first and remaining bytes
                    let Some((first, rem)) = buf.split_first_mut() else {
                        return Ok(0); // Only allowed because buf is empty
                    };

                    // We must read at least one byte, blocking if needed
                    while self.uart.fr.read().rxfe().bit_is_set() {}
                    // Read the first byte
                    *first = self.uart.dr.read().data().bits();

                    let mut num_read = 1;
                    for byte in rem {
                        // If the receive fifo is empty, return immedietly
                        if self.uart.fr.read().rxfe().bit_is_set() {
                            return Ok(num_read);
                        }

                        // Read this byte
                        *byte = self.uart.dr.read().data().bits();
                        num_read += 1;
                    }
                    Ok(num_read)
                }
            }

            // TX:

            impl<TX, RTS> embedded_io::ErrorType for Tx<$UARTX, TX, RTS> {
                type Error = Error;
            }

            impl<TX, RTS> embedded_io::WriteReady for Tx<$UARTX, TX, RTS> {
                #[inline]
                fn write_ready(&mut self) -> Result<bool, Self::Error> {
                    Ok(self.uart.fr.read().txff().bit_is_clear()) // 0 means not full, which means ready to write
                }
            }

            impl<TX, RTS> embedded_io::Write for Tx<$UARTX, TX, RTS> {
                fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                    // Get first and remaining bytes
                    let Some((first, rem)) = buf.split_first() else {
                        return Ok(0); // Only allowed because buf is empty
                    };

                    // We must transmit at least one byte, blocking if needed
                    while self.uart.fr.read().txff().bit_is_set() {}
                    // Write the first byte
                    self.uart.dr.write(|w| unsafe { w.data().bits(*first) });

                    let mut num_sent = 1;
                    // Try to write the remaining bytes
                    for byte in rem {
                        // If the transmit fifo is full, return immedietly
                        if self.uart.fr.read().txff().bit_is_set() {
                            return Ok(num_sent);
                        }

                        // Write this byte
                        self.uart.dr.write(|w| unsafe { w.data().bits(*byte) });
                        num_sent += 1;
                    }
                    Ok(num_sent)
                }

                #[inline]
                fn flush(&mut self) -> Result<(), Self::Error> {
                    while self.uart.fr.read().busy().bit_is_set() {}
                    Ok(())
                }
            }

            // RX:

            impl<RX, CTS> embedded_io::ErrorType for Rx<$UARTX, RX, CTS> {
                type Error = Error;
            }

            impl<RX, CTS> embedded_io::ReadReady for Rx<$UARTX, RX, CTS> {
                #[inline]
                fn read_ready(&mut self) -> Result<bool, Self::Error> {
                    // We're only doing RX operations here so this is safe.
                    let p = unsafe { &*$UARTX::ptr() };
                    Ok(p.fr.read().rxfe().bit_is_clear()) // 0 means not empty, which means ready to read
                }
            }

            impl<RX, CTS> embedded_io::Read for Rx<$UARTX, RX, CTS> {
                fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                    // Get first and remaining bytes
                    let Some((first, rem)) = buf.split_first_mut() else {
                        return Ok(0); // Only allowed because buf is empty
                    };

                    // We're only doing RX operations here so this is safe.
                    let p = unsafe { &*$UARTX::ptr() };

                    // We must read at least one byte, blocking if needed
                    while p.fr.read().rxfe().bit_is_set() {}
                    // Read the first byte
                    *first = p.dr.read().data().bits();

                    let mut num_read = 1;
                    for byte in rem {
                        // If the receive fifo is empty, return immedietly
                        if p.fr.read().rxfe().bit_is_set() {
                            return Ok(num_read);
                        }

                        // Read this byte
                        *byte = p.dr.read().data().bits();
                        num_read += 1;
                    }
                    Ok(num_read)
                }
            }
        )+
    }
}

/// An internal macro to help define all the different pin typestates
#[macro_export]
macro_rules! uart_pin_macro {
    ($UARTn:ident,
        cts: [$(($($ctsgpio: ident)::*, $ctsaf: ident)),*],
        // dcd: [() $(, ($($dcdgpio: ident)::*, $dcdaf: ident))*],
        // dsr: [() $(, ($($dsrgpio: ident)::*, $dsraf: ident))*],
        // dtr: [() $(, ($($dtrgpio: ident)::*, $dtraf: ident))*],
        // ri: [() $(, ($($rigpio: ident)::*, $riaf: ident))*],
        rts: [$(($($rtsgpio: ident)::*, $rtsaf: ident)),*],
        rx: [$(($($rxgpio: ident)::*, $rxaf: ident)),*],
        tx: [$(($($txgpio: ident)::*, $txaf: ident)),*],
    ) => {
        $(
            impl<T> CtsPin<$UARTn> for $($ctsgpio)::*<AlternateFunction<$ctsaf, T>>
            where
                T: OutputMode,
            {
                #[inline]
                fn enable(&mut self, uart: &mut $UARTn) {
                    uart.ctl.modify(|_, w| w.ctsen().set_bit());
                }
            }
        )*

        $(
            impl<T> RtsPin<$UARTn> for $($rtsgpio)::*<AlternateFunction<$rtsaf, T>>
            where
                T: OutputMode,
            {
                #[inline]
                fn enable(&mut self, uart: &mut $UARTn) {
                    uart.ctl.modify(|_, w| w.rtsen().set_bit());
                }
            }
        )*

        $(
            impl <T> RxPin<$UARTn> for $($rxgpio)::*<AlternateFunction<$rxaf, T>>
            where
                T: OutputMode,
            {}
        )*

        $(
            impl <T> TxPin<$UARTn> for $($txgpio)::*<AlternateFunction<$txaf, T>>
            where
                T: OutputMode,
            {}
        )*
    }
}
