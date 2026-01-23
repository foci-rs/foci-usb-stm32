// USB CDC-ACM protocol layer (port of usb_cdc.c)
//
// Handles EP0 control transfers, bulk IN/OUT, and descriptor dispatch.
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Morton Jonuschat
//
// This file may be distributed under the terms of the GNU GPLv3 license.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::descriptors::{
    self, CdcConfigDescriptor, DescriptorEntry, StringDescriptorBuf, UsbCdcLineCoding,
    UsbCtrlRequest, UsbDeviceDescriptor,
};
use crate::ep::{EP0_SIZE, EP_BULK_IN_SIZE, EP_BULK_OUT_SIZE};
use crate::otg;

// ----------------------------------------------------------------
// Wake flags (replace Klipper's sched_wake_task / sched_check_wake)
// ----------------------------------------------------------------

static EP0_WAKE: AtomicBool = AtomicBool::new(false);
static BULK_IN_WAKE: AtomicBool = AtomicBool::new(false);
static BULK_OUT_WAKE: AtomicBool = AtomicBool::new(false);

/// Signal EP0 needs processing. Called from OTG IRQ handler.
pub fn notify_ep0() {
    EP0_WAKE.store(true, Ordering::Release);
    otg::notify_wake();
}

/// Signal bulk IN needs processing. Called from OTG IRQ handler.
pub fn notify_bulk_in() {
    BULK_IN_WAKE.store(true, Ordering::Release);
    otg::notify_wake();
}

/// Signal bulk OUT needs processing. Called from OTG IRQ handler.
pub fn notify_bulk_out() {
    BULK_OUT_WAKE.store(true, Ordering::Release);
    otg::notify_wake();
}

fn check_ep0_wake() -> bool {
    EP0_WAKE.swap(false, Ordering::AcqRel)
}

fn check_bulk_in_wake() -> bool {
    BULK_IN_WAKE.swap(false, Ordering::AcqRel)
}

fn check_bulk_out_wake() -> bool {
    BULK_OUT_WAKE.swap(false, Ordering::AcqRel)
}

// ----------------------------------------------------------------
// TX buffer (bulk IN, matches Klipper's transmit_buf)
// ----------------------------------------------------------------

/// TX buffer size (matches Klipper's 192-byte transmit_buf).
const TX_BUF_SIZE: usize = 192;

// SAFETY: These statics are protected by usb_irq_disable/enable in the
// functions that access them, matching Klipper's concurrency model.
static mut TRANSMIT_BUF: [u8; TX_BUF_SIZE] = [0u8; TX_BUF_SIZE];
static mut TRANSMIT_POS: u8 = 0;

/// Write data to the USB transmit buffer. This is the primary API for
/// sending protocol responses over USB.
///
/// If the buffer is full, the data is silently dropped (matching Klipper
/// behavior in console_sendf when buffer is full).
///
/// # Safety
///
/// Must not be called from the USB ISR (reentrant access).
pub fn tx_write(data: &[u8]) {
    // SAFETY: Protected by single-threaded RTIC task context.
    // Only the protocol task calls tx_write, and the ISR only reads
    // via bulk_in_task which drains the buffer.
    unsafe {
        let tpos = TRANSMIT_POS as usize;
        if tpos + data.len() > TX_BUF_SIZE {
            // Not enough space — drop (matches Klipper's console_sendf)
            return;
        }
        TRANSMIT_BUF[tpos..tpos + data.len()].copy_from_slice(data);
        TRANSMIT_POS = (tpos + data.len()) as u8;
    }
    notify_bulk_in();
}

/// Process bulk IN transmissions. Matches `usb_bulk_in_task()` in usb_cdc.c.
fn bulk_in_task() {
    if !check_bulk_in_wake() {
        return;
    }
    // SAFETY: TRANSMIT_BUF/TRANSMIT_POS accessed from task context only.
    // The ISR sets the wake flag but does not touch these buffers.
    unsafe {
        let tpos = TRANSMIT_POS as usize;
        if tpos == 0 {
            return;
        }
        let mut max_tpos = tpos;
        if max_tpos > EP_BULK_IN_SIZE {
            max_tpos = EP_BULK_IN_SIZE;
        } else if max_tpos == EP_BULK_IN_SIZE {
            // Avoid zero-length-packets
            max_tpos = EP_BULK_IN_SIZE - 1;
        }
        let ret = otg::usb_send_bulk_in(&TRANSMIT_BUF[..max_tpos]);
        if ret <= 0 {
            return;
        }
        let ret = ret as usize;
        let needcopy = tpos - ret;
        if needcopy > 0 {
            // Move remaining data to front of buffer
            // Use copy within slice (memmove equivalent)
            let src = ret;
            for i in 0..needcopy {
                TRANSMIT_BUF[i] = TRANSMIT_BUF[src + i];
            }
            notify_bulk_in();
        }
        TRANSMIT_POS = needcopy as u8;
    }
}

// ----------------------------------------------------------------
// RX buffer (bulk OUT, matches Klipper's receive_buf)
// ----------------------------------------------------------------

/// RX buffer size (matches Klipper's 128-byte receive_buf).
const RX_BUF_SIZE: usize = 128;

// SAFETY: These statics are protected by single-threaded RTIC task context.
static mut RECEIVE_BUF: [u8; RX_BUF_SIZE] = [0u8; RX_BUF_SIZE];
static mut RECEIVE_POS: u8 = 0;

/// Get a reference to the received data available for protocol processing.
///
/// The returned slice is valid until `rx_consume()` is called.
///
/// # Safety
///
/// Must be called from the same RTIC task context that calls `poll()`.
pub fn rx_data() -> &'static [u8] {
    // SAFETY: RECEIVE_BUF/RECEIVE_POS only modified by bulk_out_task
    // which runs in the same task context as the caller.
    unsafe { &RECEIVE_BUF[..RECEIVE_POS as usize] }
}

/// Mark `len` bytes as consumed from the RX buffer. The consumed bytes
/// are removed from the front and remaining data is shifted down.
///
/// # Safety
///
/// Must be called from the same RTIC task context that calls `poll()`.
pub fn rx_consume(len: usize) {
    // SAFETY: RECEIVE_BUF/RECEIVE_POS only modified by bulk_out_task
    // which runs in the same task context as the caller.
    unsafe {
        let rpos = RECEIVE_POS as usize;
        if len >= rpos {
            RECEIVE_POS = 0;
            return;
        }
        let needcopy = rpos - len;
        for i in 0..needcopy {
            RECEIVE_BUF[i] = RECEIVE_BUF[len + i];
        }
        RECEIVE_POS = needcopy as u8;
        if needcopy > 0 {
            notify_bulk_out();
        }
    }
}

/// Process bulk OUT reception. Matches `usb_bulk_out_task()` in usb_cdc.c.
///
/// Returns true if receive data is available for protocol processing.
fn bulk_out_task() -> bool {
    if !check_bulk_out_wake() {
        // SAFETY: RECEIVE_POS only modified in this task context.
        return unsafe { RECEIVE_POS > 0 };
    }
    // SAFETY: RECEIVE_BUF/RECEIVE_POS accessed from task context only.
    unsafe {
        let rpos = RECEIVE_POS as usize;
        if rpos + EP_BULK_OUT_SIZE <= RX_BUF_SIZE {
            let ret = otg::usb_read_bulk_out(&mut RECEIVE_BUF[rpos..], EP_BULK_OUT_SIZE as u8);
            if ret > 0 {
                RECEIVE_POS = (rpos + ret as usize) as u8;
                notify_bulk_out();
            }
        } else {
            notify_bulk_out();
        }
        RECEIVE_POS > 0
    }
}

// ----------------------------------------------------------------
// EP0 control transfer state machine (port of usb_cdc.c EP0 section)
// ----------------------------------------------------------------

/// EP0 transfer direction/flags (matches UX_READ, UX_SEND, etc.)
const UX_READ: u8 = 1 << 0;
const UX_SEND: u8 = 1 << 1;
const UX_SEND_ZLP: u8 = 1 << 3;
// Note: UX_SEND_PROGMEM (1 << 2) not needed -- no PROGMEM on ARM Cortex-M.

// SAFETY: These statics are protected by the EP0 task running in a single
// RTIC task context. The ISR only sets wake flags.
static mut USB_XFER_DATA: *mut u8 = core::ptr::null_mut();
static mut USB_XFER_SIZE: u8 = 0;
static mut USB_XFER_FLAGS: u8 = 0;

/// CDC line coding state
static mut LINE_CODING: UsbCdcLineCoding = UsbCdcLineCoding {
    dw_dte_rate: 0,
    b_char_format: 0,
    b_parity_type: 0,
    b_data_bits: 0,
};

/// CDC line control state (DTR/RTS bits)
static mut LINE_CONTROL_STATE: u8 = 0;

/// Stall EP0 and clear transfer state.
fn usb_do_stall() {
    otg::usb_stall_ep0();
    // SAFETY: Protected by task context.
    unsafe {
        USB_XFER_FLAGS = 0;
    }
}

/// Execute an EP0 data transfer. Matches `usb_do_xfer()` in usb_cdc.c.
///
/// This implements the multi-packet EP0 transfer state machine:
/// - For SEND: sends data in EP0_SIZE chunks, with optional ZLP
/// - For READ: reads data directly into `data` buffer, sends status ZLP
/// - Saves state and returns if the hardware is busy (-1)
/// - Stalls on error (-2)
fn usb_do_xfer(mut data: *mut u8, mut size: u8, mut flags: u8) {
    loop {
        let xs = if size > EP0_SIZE as u8 {
            EP0_SIZE as u8
        } else {
            size
        };
        let ret: i8 = if flags & UX_READ != 0 {
            // Read directly into target buffer (matches C usb_do_xfer).
            // SAFETY: data points to a writable static with at least
            // `xs` bytes remaining (e.g. LINE_CODING).
            let buf = unsafe { core::slice::from_raw_parts_mut(data, xs as usize) };
            otg::usb_read_ep0(buf, xs)
        } else {
            // SAFETY: data points to a static descriptor or is null (ZLP).
            let slice = if xs > 0 {
                unsafe { core::slice::from_raw_parts(data, xs as usize) }
            } else {
                &[]
            };
            otg::usb_send_ep0(slice)
        };

        if ret == xs as i8 {
            // Success — advance pointer.
            // SAFETY: pointer arithmetic on valid static data.
            data = unsafe { data.add(xs as usize) };
            size -= xs;
            if size == 0 {
                // Entire transfer completed successfully
                if flags & UX_READ != 0 {
                    // Send status ZLP
                    flags = UX_SEND;
                    data = core::ptr::null_mut();
                    continue;
                }
                if xs as usize == EP0_SIZE && flags & UX_SEND_ZLP != 0 {
                    // Must send zero-length-packet
                    continue;
                }
                // SAFETY: Protected by task context.
                unsafe {
                    USB_XFER_FLAGS = 0;
                }
                notify_ep0();
                return;
            }
            continue;
        }
        if ret == -1 {
            // Interface busy - retry later
            // SAFETY: Protected by task context.
            unsafe {
                USB_XFER_DATA = data;
                USB_XFER_SIZE = size;
                USB_XFER_FLAGS = flags;
            }
            return;
        }
        // Error
        usb_do_stall();
        return;
    }
}

// ----------------------------------------------------------------
// Descriptor table (built at init time, stored in statics)
// ----------------------------------------------------------------

/// Maximum number of descriptor entries.
const MAX_DESCRIPTORS: usize = 8;

/// Static descriptor storage.
static mut DEVICE_DESC: UsbDeviceDescriptor = UsbDeviceDescriptor {
    b_length: 0,
    b_descriptor_type: 0,
    bcd_usb: 0,
    b_device_class: 0,
    b_device_sub_class: 0,
    b_device_protocol: 0,
    b_max_packet_size0: 0,
    id_vendor: 0,
    id_product: 0,
    bcd_device: 0,
    i_manufacturer: 0,
    i_product: 0,
    i_serial_number: 0,
    b_num_configurations: 0,
};
static mut CONFIG_DESC: CdcConfigDescriptor = CdcConfigDescriptor {
    config: descriptors::UsbConfigDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        w_total_length: 0,
        b_num_interfaces: 0,
        b_configuration_value: 0,
        i_configuration: 0,
        bm_attributes: 0,
        b_max_power: 0,
    },
    iface0: descriptors::UsbInterfaceDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_interface_number: 0,
        b_alternate_setting: 0,
        b_num_endpoints: 0,
        b_interface_class: 0,
        b_interface_sub_class: 0,
        b_interface_protocol: 0,
        i_interface: 0,
    },
    cdc_hdr: descriptors::UsbCdcHeaderDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_descriptor_sub_type: 0,
        bcd_cdc: 0,
    },
    cdc_acm: descriptors::UsbCdcAcmDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_descriptor_sub_type: 0,
        bm_capabilities: 0,
    },
    cdc_union: descriptors::UsbCdcUnionDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_descriptor_sub_type: 0,
        b_master_interface0: 0,
        b_slave_interface0: 0,
    },
    ep1: descriptors::UsbEndpointDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_endpoint_address: 0,
        bm_attributes: 0,
        w_max_packet_size: 0,
        b_interval: 0,
    },
    iface1: descriptors::UsbInterfaceDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_interface_number: 0,
        b_alternate_setting: 0,
        b_num_endpoints: 0,
        b_interface_class: 0,
        b_interface_sub_class: 0,
        b_interface_protocol: 0,
        i_interface: 0,
    },
    ep2: descriptors::UsbEndpointDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_endpoint_address: 0,
        bm_attributes: 0,
        w_max_packet_size: 0,
        b_interval: 0,
    },
    ep3: descriptors::UsbEndpointDescriptor {
        b_length: 0,
        b_descriptor_type: 0,
        b_endpoint_address: 0,
        bm_attributes: 0,
        w_max_packet_size: 0,
        b_interval: 0,
    },
};
static mut STR_LANG: StringDescriptorBuf = StringDescriptorBuf::lang_ids();
static mut STR_MANUFACTURER: StringDescriptorBuf = StringDescriptorBuf::from_ascii("Klipper");
static mut STR_PRODUCT: StringDescriptorBuf = StringDescriptorBuf::from_ascii("FOCI");
static mut STR_SERIAL: StringDescriptorBuf = StringDescriptorBuf::from_ascii("00000000");

static mut DESCRIPTOR_TABLE: [DescriptorEntry; MAX_DESCRIPTORS] = [
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
    DescriptorEntry {
        w_value: 0,
        w_index: 0,
        data: core::ptr::null(),
        size: 0,
    },
];
static mut DESCRIPTOR_COUNT: usize = 0;

// ----------------------------------------------------------------
// EP0 request handlers (port of usb_cdc.c request handlers)
// ----------------------------------------------------------------

/// Handle GET_DESCRIPTOR request.
fn usb_req_get_descriptor(req: &UsbCtrlRequest) {
    if req.b_request_type != descriptors::USB_DIR_IN {
        usb_do_stall();
        return;
    }
    // SAFETY: DESCRIPTOR_TABLE/DESCRIPTOR_COUNT are initialized once at
    // init time and only read here.
    #[allow(clippy::needless_range_loop)]
    let (desc, mut size) = unsafe {
        let mut found: Option<(*const u8, u8)> = None;
        let table_ptr = &raw const DESCRIPTOR_TABLE;
        let count = DESCRIPTOR_COUNT;
        for i in 0..count {
            let entry_ptr = (*table_ptr).as_ptr().add(i);
            let w_value = core::ptr::addr_of!((*entry_ptr).w_value).read();
            let w_index = core::ptr::addr_of!((*entry_ptr).w_index).read();
            if w_value == req.w_value && w_index == req.w_index {
                let data = core::ptr::addr_of!((*entry_ptr).data).read();
                let entry_size = core::ptr::addr_of!((*entry_ptr).size).read();
                found = Some((data, entry_size));
            }
        }
        match found {
            Some(v) => v,
            None => {
                usb_do_stall();
                return;
            }
        }
    };

    let mut flags = UX_SEND;
    if size > req.w_length as u8 {
        size = req.w_length as u8;
    } else if (size as u16) < req.w_length {
        flags |= UX_SEND_ZLP;
    }
    usb_do_xfer(desc as *mut u8, size, flags);
}

/// Handle SET_ADDRESS request.
fn usb_req_set_address(req: &UsbCtrlRequest) {
    if req.b_request_type != 0 || req.w_index != 0 || req.w_length != 0 {
        usb_do_stall();
        return;
    }
    otg::usb_set_address(req.w_value as u8);
}

/// Handle SET_CONFIGURATION request.
fn usb_req_set_configuration(req: &UsbCtrlRequest) {
    if req.b_request_type != 0 || req.w_value != 1 || req.w_index != 0 || req.w_length != 0 {
        usb_do_stall();
        return;
    }
    otg::usb_set_configure();
    notify_bulk_in();
    notify_bulk_out();
    usb_do_xfer(core::ptr::null_mut(), 0, UX_SEND);
}

// Katapult/CanBoot bootloader constants (from Klipper armcm_reset.c).
const KATAPULT_BOOT_ADDRESS: u32 = 0x0800_0000;
const KATAPULT_SIGNATURE: u64 = 0x2174_6f6f_426e_6143; // "CanBoot!"
const KATAPULT_REQUEST: u64 = 0x5984_E3FA_6CA1_589B; // stay in bootloader

/// True if a valid Katapult bootloader was detected at boot.
/// Set once during `detect_bootloader()`, read by `check_reboot()`.
static mut HAS_BOOTLOADER: bool = false;

/// Probe for a Katapult bootloader at KATAPULT_BOOT_ADDRESS.
/// Call once during init, before USB traffic starts.
fn detect_bootloader() {
    // SAFETY: The vector table at 0x08000000 is always readable.
    unsafe {
        let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
        let reset_handler = core::ptr::read_volatile(bl_vectors.add(1));
        let boot_sig_addr = (reset_handler - 9) as *const u64;
        if (boot_sig_addr as usize).is_multiple_of(8)
            && core::ptr::read_volatile(boot_sig_addr) == KATAPULT_SIGNATURE
        {
            HAS_BOOTLOADER = true;
        }
    }
}

/// Write the Katapult request signature and reset into the bootloader.
///
/// Matches Klipper's `canboot_reset(CANBOOT_REQUEST)` in armcm_reset.c.
fn bootloader_request() {
    // SAFETY: The vector table at KATAPULT_BOOT_ADDRESS is readable.
    // HAS_BOOTLOADER guarantees the signature is valid.
    unsafe {
        let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
        let req_sig_addr = core::ptr::read_volatile(bl_vectors) as *mut u64;
        if (req_sig_addr as usize).is_multiple_of(8) {
            cortex_m::interrupt::disable();
            core::ptr::write_volatile(req_sig_addr, KATAPULT_REQUEST);
        }
    }
    cortex_m::peripheral::SCB::sys_reset();
}

/// Check if the host is requesting a reboot into the bootloader.
/// Matches `check_reboot()` in usb_cdc.c. Only triggers if a valid
/// Katapult bootloader was detected at boot (mirrors Klipper C's
/// `CONFIG_HAVE_BOOTLOADER_REQUEST` compile-time guard).
fn check_reboot() {
    // SAFETY: LINE_CODING/LINE_CONTROL_STATE/HAS_BOOTLOADER accessed
    // from task context only.
    unsafe {
        if !HAS_BOOTLOADER {
            return;
        }
        if LINE_CODING.dw_dte_rate == 1200 && (LINE_CONTROL_STATE & 0x01) == 0 {
            bootloader_request();
        }
    }
}

/// Handle SET_LINE_CODING request.
fn usb_req_set_line_coding(req: &UsbCtrlRequest) {
    let line_coding_size = core::mem::size_of::<UsbCdcLineCoding>() as u16;
    if req.b_request_type != 0x21
        || req.w_value != 0
        || req.w_index != 0
        || req.w_length != line_coding_size
    {
        usb_do_stall();
        return;
    }
    // SAFETY: LINE_CODING only accessed from task context.
    let ptr = &raw mut LINE_CODING;
    usb_do_xfer(ptr as *mut u8, line_coding_size as u8, UX_READ);
    check_reboot();
}

/// Handle GET_LINE_CODING request.
fn usb_req_get_line_coding(req: &UsbCtrlRequest) {
    let line_coding_size = core::mem::size_of::<UsbCdcLineCoding>() as u16;
    if req.b_request_type != 0xA1
        || req.w_value != 0
        || req.w_index != 0
        || req.w_length < line_coding_size
    {
        usb_do_stall();
        return;
    }
    // SAFETY: LINE_CODING only accessed from task context.
    let ptr = &raw const LINE_CODING;
    usb_do_xfer(ptr as *mut u8, line_coding_size as u8, UX_SEND);
}

/// Handle SET_CONTROL_LINE_STATE request.
fn usb_req_set_line(req: &UsbCtrlRequest) {
    if req.b_request_type != 0x21 || req.w_index != 0 || req.w_length != 0 {
        usb_do_stall();
        return;
    }
    // SAFETY: LINE_CONTROL_STATE is only accessed from task context.
    unsafe {
        LINE_CONTROL_STATE = req.w_value as u8;
    }
    usb_do_xfer(core::ptr::null_mut(), 0, UX_SEND);
    check_reboot();
}

/// EP0 state machine: read a setup packet and dispatch.
/// Matches `usb_state_ready()` in usb_cdc.c.
fn usb_state_ready() {
    let mut req_buf = [0u8; 8];
    let ret = otg::usb_read_ep0_setup(&mut req_buf, 8);
    if ret != 8 {
        return;
    }

    // SAFETY: UsbCtrlRequest is repr(C, packed) and exactly 8 bytes.
    // The buffer is properly aligned for byte-level access.
    let req: UsbCtrlRequest = unsafe { core::ptr::read_unaligned(req_buf.as_ptr().cast()) };

    match req.b_request {
        descriptors::USB_REQ_GET_DESCRIPTOR => usb_req_get_descriptor(&req),
        descriptors::USB_REQ_SET_ADDRESS => usb_req_set_address(&req),
        descriptors::USB_REQ_SET_CONFIGURATION => usb_req_set_configuration(&req),
        descriptors::USB_CDC_REQ_SET_LINE_CODING => usb_req_set_line_coding(&req),
        descriptors::USB_CDC_REQ_GET_LINE_CODING => usb_req_get_line_coding(&req),
        descriptors::USB_CDC_REQ_SET_CONTROL_LINE_STATE => usb_req_set_line(&req),
        _ => usb_do_stall(),
    }
}

/// EP0 task: process pending EP0 transfers or read new setup packets.
/// Matches `usb_ep0_task()` in usb_cdc.c.
fn ep0_task() {
    if !check_ep0_wake() {
        return;
    }
    // SAFETY: USB_XFER_FLAGS/USB_XFER_DATA/USB_XFER_SIZE accessed
    // from task context only.
    unsafe {
        if USB_XFER_FLAGS != 0 {
            usb_do_xfer(USB_XFER_DATA, USB_XFER_SIZE, USB_XFER_FLAGS);
        } else {
            usb_state_ready();
        }
    }
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

/// Initialize the CDC layer with descriptor data.
///
/// Must be called before `poll()`. Typically called right after `otg::init()`.
///
/// # Safety
///
/// Must be called exactly once, before any other CDC function.
pub unsafe fn init(vid: u16, pid: u16, manufacturer: &str, product: &str, serial: &str) {
    // Check for Katapult bootloader (must happen before USB traffic).
    detect_bootloader();

    // Build descriptors
    DEVICE_DESC = descriptors::build_device_descriptor(vid, pid);
    CONFIG_DESC = descriptors::build_config_descriptor();
    STR_LANG = StringDescriptorBuf::lang_ids();
    STR_MANUFACTURER = StringDescriptorBuf::from_ascii(manufacturer);
    STR_PRODUCT = StringDescriptorBuf::from_ascii(product);
    STR_SERIAL = StringDescriptorBuf::from_ascii(serial);

    // Build descriptor lookup table using raw pointers to avoid
    // creating references to mutable statics.
    let mut idx = 0;

    // Device descriptor
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_DEVICE as u16) << 8,
        w_index: 0,
        data: (&raw const DEVICE_DESC) as *const u8,
        size: core::mem::size_of::<UsbDeviceDescriptor>() as u8,
    };
    idx += 1;

    // Config descriptor
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_CONFIG as u16) << 8,
        w_index: 0,
        data: (&raw const CONFIG_DESC) as *const u8,
        size: core::mem::size_of::<CdcConfigDescriptor>() as u8,
    };
    idx += 1;

    // String 0: Language IDs
    let (lang_ptr, lang_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const STR_LANG);
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_STRING as u16) << 8,
        w_index: 0,
        data: lang_ptr,
        size: lang_len,
    };
    idx += 1;

    // String 1: Manufacturer
    let (mfr_ptr, mfr_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const STR_MANUFACTURER);
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_STRING as u16) << 8
            | descriptors::USB_STR_ID_MANUFACTURER as u16,
        w_index: descriptors::USB_LANGID_ENGLISH_US,
        data: mfr_ptr,
        size: mfr_len,
    };
    idx += 1;

    // String 2: Product
    let (prod_ptr, prod_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const STR_PRODUCT);
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_STRING as u16) << 8 | descriptors::USB_STR_ID_PRODUCT as u16,
        w_index: descriptors::USB_LANGID_ENGLISH_US,
        data: prod_ptr,
        size: prod_len,
    };
    idx += 1;

    // String 3: Serial
    let (ser_ptr, ser_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const STR_SERIAL);
    DESCRIPTOR_TABLE[idx] = DescriptorEntry {
        w_value: (descriptors::USB_DT_STRING as u16) << 8 | descriptors::USB_STR_ID_SERIAL as u16,
        w_index: descriptors::USB_LANGID_ENGLISH_US,
        data: ser_ptr,
        size: ser_len,
    };
    idx += 1;

    DESCRIPTOR_COUNT = idx;

    // Reset transfer state
    USB_XFER_DATA = core::ptr::null_mut();
    USB_XFER_SIZE = 0;
    USB_XFER_FLAGS = 0;
    TRANSMIT_POS = 0;
    RECEIVE_POS = 0;
    LINE_CODING = UsbCdcLineCoding::default();
    LINE_CONTROL_STATE = 0;
}

/// Update the serial number descriptor from a chip ID.
///
/// Call after `init()` if using hardware chip ID instead of a fixed serial string.
///
/// # Safety
///
/// Must be called before USB enumeration completes (before the host
/// reads the serial descriptor).
pub unsafe fn set_serial_from_chip_id(id: &[u8], strlen: usize) {
    let serial_ptr = &raw mut STR_SERIAL;
    descriptors::fill_serial_from_chip_id(&mut *serial_ptr, strlen, id);
    // Update the descriptor table entry for the serial string
    let expected_wvalue =
        (descriptors::USB_DT_STRING as u16) << 8 | descriptors::USB_STR_ID_SERIAL as u16;
    let (ser_ptr, ser_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const STR_SERIAL);
    let table_ptr = &raw mut DESCRIPTOR_TABLE;
    let count = DESCRIPTOR_COUNT;
    #[allow(clippy::needless_range_loop)]
    for i in 0..count {
        let entry_ptr = (*table_ptr).as_mut_ptr().add(i);
        let w_value = core::ptr::addr_of!((*entry_ptr).w_value).read();
        if w_value == expected_wvalue {
            core::ptr::addr_of_mut!((*entry_ptr).data).write(ser_ptr);
            core::ptr::addr_of_mut!((*entry_ptr).size).write(ser_len);
            break;
        }
    }
}

/// Poll all USB tasks: EP0, bulk IN, bulk OUT.
///
/// Returns true if RX data is available for protocol processing.
/// Call this from the RTIC USB task whenever `check_wake()` returns true.
pub fn poll() -> bool {
    ep0_task();
    bulk_in_task();
    bulk_out_task()
}

/// Check if the USB device is configured (endpoints active).
/// Returns true after the host sends SET_CONFIGURATION.
pub fn is_configured() -> bool {
    otg::is_bulk_in_configured()
}

/// Signal shutdown — wake all tasks so they can drain.
/// Matches `usb_shutdown()` in usb_cdc.c.
pub fn shutdown() {
    notify_bulk_in();
    notify_bulk_out();
    notify_ep0();
}
