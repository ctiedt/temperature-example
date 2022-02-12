all: temperature-example.bin

temperature-example.bin: temperature-example
	arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabi/release/temperature-example build/temperature-example.bin

temperature-example: src/*.rs
	cargo build --release

clean:
	cargo clean
	rm -rf build/*

prepare:
	rustup override set nightly
	rustup target install thumbv7em-none-eabi
	mkdir build