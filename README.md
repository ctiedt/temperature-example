## Project Setup

Install the `thumbv7em-none-eabi` target using
`rustup target install thumbv7em-none-eabi`.

Make sure that you have the `binutils-arm-none-eabi` package installed.

## Building and flashing

Build the project using `cargo build --release`. Then run `arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabi/release/node node.bin`.

The `STM32F401RE` board should appear as a USB drive on the PC.
Copy the `.bin` file onto it.
