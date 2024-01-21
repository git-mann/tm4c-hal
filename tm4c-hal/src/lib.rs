//! Generic implementation code for both TM4C123 and TM4C129.

#![no_std]
#![deny(missing_docs, warnings)]
#![allow(deprecated)]

pub mod bb;
pub mod delay;
pub mod eeprom;
pub mod gpio;
pub mod i2c;
pub mod serial;
pub mod sysctl;
pub mod time;
