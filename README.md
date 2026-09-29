# foci-usb-stm32

STM32 USB OTG CDC-ACM transport for Klipper protocol (port of usbotg.c + usb_cdc.c).

Part of the [FOCI](https://github.com/foci-rs/foci) project — a
Klipper/Kalico-compatible MCU firmware family for STM32 boards with
TMC4671 FOC servo controller ICs.

## Status

Pre-1.0; APIs may change without notice. Used in production firmware
on the OpenFFBoard test rig but not yet versioned for external
consumers.

## Use

Add as a git dependency in your `Cargo.toml`:

```toml
foci-usb-stm32 = { git = "https://github.com/foci-rs/foci-usb-stm32" }
```

## License

GPL-3.0-or-later. See `LICENSE` at the repo root for the full text.
