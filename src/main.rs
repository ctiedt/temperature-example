#![no_std]
#![no_main]
#![feature(panic_info_message)]
#![feature(alloc_error_handler)]

extern crate alloc;

use core::panic::PanicInfo;

use alloc::vec;
//use cortex_m;
use cortex_m_rt::entry;

use hal::{
    gpio::*,
    pac::USART2,
    prelude::*,
    serial::{config::Config, Serial, Tx},
};

use onewire::{ds18b20, DeviceSearch, OneWire, OpenDrainOutput, DS18B20};
use stm32f4xx_hal as hal;

mod io;
use io::_print;
mod mem;
use mem::ALLOCATOR;

pub static mut DELAY: Option<hal::delay::Delay> = None;
pub static mut TX: Option<Tx<USART2>> = None;

fn blink_led(led: &mut ErasedPin<Output<PushPull>>) {
    println!("Blink");
    led.set_high();
    wait(100);
    led.set_low();
    wait(200);
}

fn blink_temperature(led: &mut ErasedPin<Output<PushPull>>, mut temperature: f32) {
    let mut digits = vec![];
    while temperature > 1.0 {
        let digit = temperature as i32 % 10;
        temperature /= 10.0;
        digits.push(digit);
    }
    let digits = digits.into_iter().rev();

    for digit in digits {
        for _ in 0..digit {
            blink_led(led);
        }
        wait(500);
    }
}

fn read_temperature<ODO: OpenDrainOutput>(
    ds18b20: &DS18B20,
    mut wire: &mut OneWire<ODO>,
) -> Result<f32, &'static str> {
    let resolution = ds18b20
        .measure_temperature(&mut wire, unsafe { DELAY.as_mut().unwrap() })
        .unwrap();

    wait(resolution.time_ms().into());

    if let Ok(temperature) = ds18b20.read_temperature(&mut wire, unsafe { DELAY.as_mut().unwrap() })
    {
        let temperature = temperature as f32 * 0.0625;
        println!("temperature: {}", temperature);
        return Ok(temperature);
    }
    Err("Couldn't read temperature.")
}

#[entry]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024;
    unsafe { ALLOCATOR.init(start, size) }

    let device_peripherals = hal::pac::Peripherals::take().unwrap();
    let core_peripherals = cortex_m::peripheral::Peripherals::take().unwrap();

    let gpioa = device_peripherals.GPIOA.split();

    let rcc = device_peripherals.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.mhz()).freeze();
    unsafe {
        DELAY = hal::delay::Delay::new(core_peripherals.SYST, &clocks).into();
    }

    let tx_pin = gpioa.pa2.into_alternate();

    // configure serial
    let tx = Serial::tx(
        device_peripherals.USART2,
        tx_pin,
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap();

    unsafe { TX = Some(tx) };

    println!("Setup done");

    // OneWire Setup
    let one = gpioa
        .pa0
        .into_open_drain_output_in_state(PinState::High)
        .erase_number();

    let led_pin = gpioa.pa8.into_push_pull_output();

    let mut wire = OneWire::new(one, true);

    wire.reset(unsafe { DELAY.as_mut().unwrap() }).unwrap();

    let mut search = DeviceSearch::new().into_iter(&mut wire, unsafe { DELAY.as_mut().unwrap() });

    let device = search
        .find(|device| device.as_ref().unwrap().address[0] == ds18b20::FAMILY_CODE)
        .unwrap()
        .unwrap();

    let sensor = DS18B20::new(device).unwrap();

    let mut led_pin = led_pin.erase();
    loop {
        let temperature = match read_temperature(&sensor, &mut wire) {
            Ok(it) => it,
            Err(_) => {
                println!("Couldn't read temperature.");
                continue;
            }
        };
        blink_temperature(&mut led_pin, temperature);
        wait(1000);
    }
}

pub fn wait(millis: u32) {
    unsafe {
        DELAY.as_mut().unwrap().delay_ms(millis);
    }
}

pub fn wait_micros(micros: u32) {
    unsafe {
        DELAY.as_mut().unwrap().delay_us(micros);
    }
}

#[panic_handler]
fn handle_panic(panic_info: &PanicInfo) -> ! {
    println!("💥 The program panicked. THIS IS NOT A DRILL!");
    if let Some(location) = panic_info.location() {
        println!(
            "📌 Location:  {}:{}:{}",
            location.file(),
            location.line(),
            location.column(),
        );
    }
    if let Some(message) = panic_info.message() {
        println!("{}", message);
    }

    loop {}
}
