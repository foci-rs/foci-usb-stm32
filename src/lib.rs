// USB OTG CDC-ACM driver for Klipper protocol
//
// Faithful port of Klipper's src/stm32/usbotg.c and src/generic/usb_cdc.c
// to Rust, for use in the FOCI firmware.
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Morton Jonuschat
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#![no_std]

pub mod cdc;
pub mod descriptors;
pub mod otg;
mod sync;
#[cfg(feature = "trace")]
pub mod trace;
mod tx_ring;

/// Endpoint constants (from usb_cdc_ep.h)
pub mod ep {
    /// EP0 max packet size. Klipper uses 16, not the typical 64.
    pub const EP0_SIZE: usize = 16;

    /// Bulk IN endpoint number (device-to-host protocol data).
    pub const EP_BULK_IN: usize = 1;

    /// Bulk OUT endpoint number (host-to-device protocol data).
    pub const EP_BULK_OUT: usize = 2;

    /// ACM interrupt endpoint number.
    pub const EP_ACM: usize = 3;

    /// ACM endpoint packet size.
    pub const EP_ACM_SIZE: usize = 8;

    /// Bulk OUT endpoint packet size.
    pub const EP_BULK_OUT_SIZE: usize = 64;

    /// Bulk IN endpoint packet size.
    pub const EP_BULK_IN_SIZE: usize = 64;

    /// Trace bulk IN endpoint number.
    ///
    /// Reuse endpoint number 2 in the opposite direction from CDC bulk OUT.
    /// STM32 OTG has separate IN and OUT endpoint registers; this keeps the
    /// trace interface within the F407's endpoint-number range.
    #[cfg(feature = "trace")]
    pub const EP_TRACE_IN: usize = EP_BULK_OUT;

    /// Trace bulk IN endpoint packet size.
    #[cfg(feature = "trace")]
    pub const EP_TRACE_IN_SIZE: usize = 64;
}

/// USB peripheral configuration.
pub struct UsbConfig {
    /// Base address of the USB OTG peripheral.
    pub base_addr: usize,
    /// IRQ number for the USB OTG interrupt.
    pub irq_num: u16,
    /// Turnaround time (TRDT field in GUSBCFG). 6 for both F4 and H7.
    pub trdt: u8,
    /// Use VBUS B-session valid override (true for F446/H7/F7, false for F4).
    pub vbus_detection: bool,
    /// Enable double-buffer TX mode for bulk IN.
    pub double_buffer_tx: bool,
    /// USB Vendor ID.
    pub vid: u16,
    /// USB Product ID.
    pub pid: u16,
    /// Manufacturer string (ASCII, max 31 chars).
    pub manufacturer: &'static str,
    /// Product string (ASCII, max 31 chars).
    pub product: &'static str,
    /// Serial number string (ASCII, max 31 chars). Ignored if chip ID is used.
    pub serial: &'static str,
}

/// Initialize the USB subsystem.
///
/// The caller must enable the USB peripheral clock and configure GPIO pins
/// before calling this function.
///
/// # Safety
///
/// Must be called exactly once before any other function in this crate.
/// `config.base_addr` must be the valid base address of an STM32 USB OTG
/// peripheral.
pub unsafe fn usb_init(config: &UsbConfig) {
    // SAFETY: caller upholds the safety contract of `usb_init`.
    unsafe {
        otg::init(&otg::OtgConfig {
            base_addr: config.base_addr,
            irq_num: config.irq_num,
            trdt: config.trdt,
            vbus_detection: config.vbus_detection,
            double_buffer_tx: config.double_buffer_tx,
        });
        cdc::init(
            config.vid,
            config.pid,
            config.manufacturer,
            config.product,
            config.serial,
        );
    }
}

/// USB OTG IRQ handler. Call this from the RTIC interrupt binding.
pub fn irq_handler() {
    otg::irq_handler();
}

/// Poll all USB tasks. Returns true if RX data is available.
///
/// Call this from the RTIC task whenever `check_wake()` returns true.
pub fn poll() -> bool {
    cdc::poll()
}

/// Get a reference to received protocol data.
pub fn rx_data() -> &'static [u8] {
    cdc::rx_data()
}

/// Mark `len` bytes of RX data as consumed.
pub fn rx_consume(len: usize) {
    cdc::rx_consume(len);
}

/// Write data to the USB transmit buffer.
pub fn tx_write(data: &[u8]) {
    cdc::tx_write(data);
}

/// Write one packet to the trace bulk IN endpoint.
///
/// Returns the number of bytes accepted by the USB controller, or -1 if the
/// trace endpoint is busy and the caller should retry later.
#[cfg(feature = "trace")]
pub fn trace_write(data: &[u8]) -> i8 {
    trace::send_trace_in(data)
}

/// Check if the USB device is configured.
pub fn is_configured() -> bool {
    cdc::is_configured()
}

/// Check and clear the wake flag. Returns true if USB needs polling.
pub fn check_wake() -> bool {
    otg::check_wake()
}

/// Set the serial number from a hardware chip ID.
///
/// Call after `usb_init()` but before USB enumeration completes.
///
/// # Safety
///
/// Must be called before the host reads the serial number descriptor.
pub unsafe fn set_serial_from_chip_id(id: &[u8], strlen: usize) {
    // SAFETY: caller upholds the safety contract of `set_serial_from_chip_id`.
    unsafe { cdc::set_serial_from_chip_id(id, strlen) };
}
