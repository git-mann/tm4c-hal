//! Common I2C code for TM4C123 and TM4C129

use embedded_hal::i2c;

/// I2C error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Bus Busy
    BusBusy,

    /// Arbitration loss
    Arbitration,

    /// Missing Data ACK
    DataAck,

    /// Missing Address ACK
    AdrAck,

    /// I2C Timeout
    Timeout,
}

impl i2c::Error for Error {
    #[inline]
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::BusBusy => i2c::ErrorKind::Bus,
            Error::Arbitration => i2c::ErrorKind::ArbitrationLoss,
            Error::DataAck => i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Data),
            Error::AdrAck => i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Address),
            Error::Timeout => i2c::ErrorKind::Other,
        }
    }
}

#[macro_export]
/// Implements the traits for an I2C peripheral
macro_rules! i2c_pins {
    ($I2Cn:ident,
        scl: [$(($($sclgpio: ident)::*, $sclaf: ident)),*],
        sda: [$(($($sdagpio: ident)::*, $sdaaf: ident)),*],
    ) => {
        $(
            impl<T> SclPin<$I2Cn> for $($sclgpio)::*<AlternateFunction<$sclaf, T>>
            where
                T: OutputMode,
            {}
        )*

        $(
            impl<T> SdaPin<$I2Cn> for $($sdagpio)::*<AlternateFunction<$sdaaf, T>>
            where
                T: OutputMode,
            {}
        )*
    }
}

#[macro_export]
/// Spins until the controler is ready (mcs.busy is clear) and optionally on
/// another field of the mcs register until it is clear or set (depending on op
/// parameter).
macro_rules! i2c_busy_wait {
    ($i2c:expr $(, $field:ident, $op:ident)? ) => {{
        // in 'release' builds, the time between setting the `run` bit and checking the `busy`
        // bit is too short and the `busy` bit is not reliably set by the time you get there,
        // it can take up to 8 clock cycles for the `run` to begin so this delay allows time
        // for that hardware synchronization
        delay(8);

        // Allow 1,000 clock cycles before we timeout. At 100 kHz, this is 10 ms.
        $i2c.mclkocnt
            .write(|w| unsafe { w.cntl().bits((1_000 >> 4) as u8) });

        let mcs = loop {
            let mcs = $i2c.mcs.read();

            if mcs.busy().bit_is_clear() {
                break mcs;
            }
        };


        if mcs.clkto().bit_is_set() {
            return Err(Error::Timeout)
        } else if mcs.arblst().bit_is_set() {
            return Err(Error::Arbitration)
        } else if mcs.error().bit_is_set() {
            if mcs.adrack().bit_is_set() {
                return Err(Error::AdrAck);
            } else { // if mcs.datack().bit_is_set() {
                return Err(Error::DataAck);
            }
        }

        $( loop {
            if mcs.clkto().bit_is_set() {
                return Err(Error::Timeout)
            } else if mcs.arblst().bit_is_set() {
                return Err(Error::Arbitration)
            } else if mcs.error().bit_is_set() {
                if mcs.adrack().bit_is_set() {
                    return Err(Error::AdrAck);
                } else { // if mcs.datack().bit_is_set() {
                    return Err(Error::DataAck);
                }
            } else if mcs.$field().$op() {
                break;
            } else {
                // try again
            }
        };)?

        Ok(())
    }};
}

#[macro_export]
/// Implements embedded-hal for an TM4C I2C peripheral
#[allow(clippy::crate_in_macro_def)]
macro_rules! i2c_hal {
    ($($I2CX:ident: ($powerDomain:ident, $i2cX:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                    clocks: &Clocks,
                    pc: &sysctl::PowerControl,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    sysctl::control_power(
                        pc, sysctl::Domain::$powerDomain,
                        sysctl::RunMode::Run, sysctl::PowerState::On);
                    sysctl::reset(pc, sysctl::Domain::$powerDomain);

                    // set Master Function Enable, and clear other bits.
                    i2c.mcr.write(|w| w.mfe().set_bit());

                    // Write TimerPeriod configuration and clear other bits.
                    let freq = freq.into().0;
                    let tpr = ((clocks.sysclk.0/(2*10*freq))-1) as u8;

                    i2c.mtpr.write(|w| unsafe {w.tpr().bits(tpr)});

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> embedded_hal::i2c::ErrorType for I2c<$I2CX, PINS> {
                type Error = Error;
            }

            impl<PINS> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2c<$I2CX, PINS> {
                fn transaction(
                    &mut self,
                    address: embedded_hal::i2c::SevenBitAddress,
                    operations: &mut [embedded_hal::i2c::Operation<'_>],
                ) -> Result<(), Self::Error> {
                    let mut prev_operation = None;
                    let num_operations = operations.len();

                    for (op_number, operation) in operations.into_iter().enumerate() {
                        let is_last_op = op_number == (num_operations - 1);

                        match operation {
                            embedded_hal::i2c::Operation::Read(buffer) => {
                                let num_bytes = buffer.len();
                                let Some((first_buf, rem_buf)) = buffer.split_first_mut() else {
                                    // Empty operation does nothing
                                    debug_assert!(
                                        false,
                                        "Read operation has empty slice! Probably not intended."
                                    );
                                    continue;
                                };

                                match prev_operation {
                                    // If last operation was a Write, we need to send address and repeated start
                                    Some(&mut embedded_hal::i2c::Operation::Write(_)) => {
                                        // Write Slave address and set Receive bit
                                        self.i2c
                                            .msa
                                            .write(|w| unsafe { w.sa().bits(address).rs().set_bit() });

                                        // If single receive
                                        if rem_buf.is_empty() {
                                            // emit Repeated START and STOP for single receive
                                            self.i2c.mcs.write(|w| {
                                                if is_last_op {
                                                    w.stop().set_bit();
                                                }
                                                w.run().set_bit().start().set_bit()
                                            });
                                        } else {
                                            // emit Repeated START
                                            self.i2c
                                                .mcs
                                                .write(|w| w.run().set_bit().start().set_bit().ack().set_bit());
                                        }
                                    }
                                    // If last operation was also a Read, we don't need to send address or repeated start
                                    Some(&mut embedded_hal::i2c::Operation::Read(_)) => {
                                        // If single receive
                                        if rem_buf.is_empty() {
                                            // Set run and start, and stop if last operation
                                            self.i2c.mcs.write(|w| {
                                                if is_last_op {
                                                    w.stop().set_bit();
                                                }
                                                w.run().set_bit()
                                            });
                                        } else {
                                            // Set run, and ack
                                            self.i2c.mcs.write(|w| w.run().set_bit().ack().set_bit());
                                        }
                                    }
                                    // If there was no last operation, we need to send address and start
                                    None => {
                                        // Write Slave address and set Receive bit
                                        self.i2c
                                            .msa
                                            .write(|w| unsafe { w.sa().bits(address).rs().set_bit() });

                                        i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;

                                        // If single receive
                                        if rem_buf.is_empty() {
                                            // Set run and start, and stop if last operation
                                            self.i2c.mcs.write(|w| {
                                                if is_last_op {
                                                    w.stop().set_bit();
                                                }
                                                w.run().set_bit().start().set_bit()
                                            });
                                        } else {
                                            // Set run, start, and ack
                                            self.i2c
                                                .mcs
                                                .write(|w| w.start().set_bit().run().set_bit().ack().set_bit());
                                        }
                                    }
                                }
                                i2c_busy_wait!(self.i2c)?;
                                *first_buf = self.i2c.mdr.read().data().bits();

                                if let Some((last, buf)) = rem_buf.split_last_mut() {
                                    for byte in buf {
                                        self.i2c.mcs.write(|w| w.run().set_bit().ack().set_bit());
                                        i2c_busy_wait!(self.i2c)?;
                                        *byte = self.i2c.mdr.read().data().bits();
                                    }

                                    self.i2c.mcs.write(|w| {
                                        if is_last_op {
                                            w.stop().set_bit();
                                        } else {
                                            w.ack().set_bit(); // TODO: is this correct?
                                        }
                                        w.run().set_bit()
                                    });
                                    i2c_busy_wait!(self.i2c)?;
                                    *last = self.i2c.mdr.read().data().bits();
                                }

                                i2c_busy_wait!(self.i2c)?;
                                prev_operation = Some(operation);
                            }
                            embedded_hal::i2c::Operation::Write(bytes) => {
                                let num_bytes = bytes.len();
                                let Some((first_byte, rem_bytes)) = bytes.split_first() else {
                                    // Empty operation does nothing
                                    debug_assert!(
                                        false,
                                        "Write operation has empty slice! Probably not intended."
                                    );
                                    continue;
                                };

                                match prev_operation {
                                    // If last operation was also a Write, we don't need to send address or repeated start
                                    Some(&mut embedded_hal::i2c::Operation::Write(_)) => {
                                        // send first byte
                                        self.i2c.mdr.write(|w| unsafe { w.data().bits(*first_byte) });
                                        i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                                        // If single byte transfer and last operation, set STOP
                                        if rem_bytes.is_empty() && is_last_op {
                                            self.i2c.mcs.write(|w| w.run().set_bit().stop().set_bit());
                                        }
                                    }
                                    // If last operation was a Read, we need to send address and repeated start
                                    Some(&mut embedded_hal::i2c::Operation::Read(_)) => {
                                        self.i2c
                                            .msa
                                            .write(|w| unsafe { w.sa().bits(address).rs().clear_bit() });

                                        // send first byte
                                        self.i2c.mdr.write(|w| unsafe { w.data().bits(*first_byte) });
                                        i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                                        // emit Repeated START
                                        self.i2c.mcs.write(|w| {
                                            // If single byte transfer and last operation, set STOP
                                            if rem_bytes.is_empty() && is_last_op {
                                                w.stop().set_bit();
                                            }
                                            w.run().set_bit().start().set_bit().ack().set_bit()
                                        });
                                    }
                                    // If there was no last operation, we need to send address and start
                                    None => {
                                        self.i2c
                                            .msa
                                            .write(|w| unsafe { w.sa().bits(address).rs().clear_bit() });

                                        // send first byte
                                        self.i2c.mdr.write(|w| unsafe { w.data().bits(*first_byte) });
                                        i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                                        self.i2c.mcs.write(|w| {
                                            // If single byte transfer and last operation, set STOP
                                            if rem_bytes.is_empty() && is_last_op {
                                                w.stop().set_bit();
                                            }
                                            w.start().set_bit().run().set_bit()
                                        });
                                    }
                                }

                                for (i, byte) in rem_bytes.iter().enumerate() {
                                    i2c_busy_wait!(self.i2c)?;

                                    // Put next byte in data register
                                    self.i2c.mdr.write(|w| unsafe { w.data().bits(*byte) });

                                    // Send RUN command (Burst continue)
                                    self.i2c.mcs.write(|w| {
                                        // Set STOP on last byte (and last operation)
                                        if (i + 1) == (num_bytes - 1) && is_last_op {
                                            w.stop().set_bit();
                                        }
                                        w.run().set_bit()
                                    });
                                }

                                i2c_busy_wait!(self.i2c)?;
                                prev_operation = Some(operation);
                            }
                        }
                    }

                    Ok(())
                }
            }
        )+
    }
}
