use alloc::string::ToString;
use core::fmt::{Arguments, Write};

use crate::TX;

pub fn _print(message: Arguments) {
    let msg_string = message.to_string();
    let tx = unsafe { TX.as_mut().unwrap() };
    tx.write_str(&msg_string).unwrap();
}

#[macro_export]
macro_rules! println {
    ($fmt:expr) => (print!(concat!($fmt, "\r\n")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\r\n"), $($arg)*));
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => (_print(format_args!($($arg)*)));
}
