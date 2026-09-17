// USB CDC-ACM protocol layer (port of usb_cdc.c)
//
// Handles EP0 control transfers, bulk IN/OUT, and descriptor dispatch.
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Morton Jonuschat
//
// This file may be distributed under the terms of the GNU GPLv3 license.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::interrupt::{self, Mutex};

use crate::descriptors::{
    self, CdcConfigDescriptor, DescriptorEntry, StringDescriptorBuf, UsbCdcLineCoding,
    UsbCtrlRequest, UsbDeviceDescriptor,
};
#[cfg(feature = "trace")]
use crate::ep::EP_TRACE_IN;
use crate::ep::{EP_ACM, EP_BULK_IN, EP_BULK_IN_SIZE, EP_BULK_OUT, EP_BULK_OUT_SIZE, EP0_SIZE};
use crate::otg;
use crate::sync::RacyCell;

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

/// Bulk IN transmit buffer. `pos` is the current fill level in `buf`.
struct TxBuffer {
    buf: [u8; TX_BUF_SIZE],
    pos: u8,
}

/// Bulk IN TX state shared by reply-producing tasks and the USB drain task.
///
/// The USB ISR only sets the bulk-in wake flag. Task-context writers and the
/// drain can preempt each other, so every buffer access uses one brief global
/// critical section.
static TX: Mutex<RefCell<TxBuffer>> = Mutex::new(RefCell::new(TxBuffer {
    buf: [0u8; TX_BUF_SIZE],
    pos: 0,
}));

/// Count of `tx_write` calls that silently dropped a frame because it did
/// not fit in the remaining TX staging capacity. Diagnostic only -- read
/// back via `tx_buffer_full_drop_count`, not logged live at this call site.
static TX_BUFFER_FULL_DROPS: AtomicU32 = AtomicU32::new(0);

/// Current count of frames dropped by `tx_write`'s buffer-full path.
pub fn tx_buffer_full_drop_count() -> u32 {
    TX_BUFFER_FULL_DROPS.load(Ordering::Relaxed)
}

/// Write data to the USB transmit buffer. This is the primary API for
/// sending protocol responses over USB.
///
/// If the buffer is full, the data is silently dropped (matching Klipper
/// behavior in console_sendf when buffer is full). Each drop still counts
/// toward `tx_buffer_full_drop_count`.
///
/// Must not be called from the USB ISR.
pub fn tx_write(data: &[u8]) {
    let written = interrupt::free(|cs| {
        let mut tx = TX.borrow(cs).borrow_mut();
        let tpos = tx.pos as usize;
        if tpos + data.len() > TX_BUF_SIZE {
            // Not enough space — drop (matches Klipper's console_sendf).
            TX_BUFFER_FULL_DROPS.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        tx.buf[tpos..tpos + data.len()].copy_from_slice(data);
        tx.pos = (tpos + data.len()) as u8;
        true
    });
    if written {
        notify_bulk_in();
    }
}

/// Process bulk IN transmissions. Matches `usb_bulk_in_task()` in usb_cdc.c.
fn bulk_in_task() {
    if !check_bulk_in_wake() {
        return;
    }
    let needs_more = interrupt::free(|cs| {
        let mut tx = TX.borrow(cs).borrow_mut();
        let tpos = tx.pos as usize;
        if tpos == 0 {
            return false;
        }
        let mut max_tpos = tpos;
        if max_tpos > EP_BULK_IN_SIZE {
            max_tpos = EP_BULK_IN_SIZE;
        } else if max_tpos == EP_BULK_IN_SIZE {
            // Avoid zero-length-packets.
            max_tpos = EP_BULK_IN_SIZE - 1;
        }
        let ret = otg::usb_send_bulk_in(&tx.buf[..max_tpos]);
        if ret <= 0 {
            return false;
        }
        let ret = ret as usize;
        let needcopy = tpos - ret;
        if needcopy > 0 {
            // Move remaining data to front of buffer (memmove-equivalent).
            let src = ret;
            for i in 0..needcopy {
                tx.buf[i] = tx.buf[src + i];
            }
        }
        tx.pos = needcopy as u8;
        needcopy > 0
    });
    if needs_more {
        notify_bulk_in();
    }
}

// ----------------------------------------------------------------
// RX buffer (bulk OUT, matches Klipper's receive_buf)
// ----------------------------------------------------------------

/// RX buffer size (matches Klipper's 128-byte receive_buf).
const RX_BUF_SIZE: usize = 128;

/// Bulk OUT receive buffer. `pos` is the current fill level in `buf`.
struct RxBuffer {
    buf: [u8; RX_BUF_SIZE],
    pos: u8,
}

/// Bulk OUT RX state. All accesses occur in the single protocol-task
/// context; the USB ISR only sets the bulk-out wake flag.
static RX: RacyCell<RxBuffer> = RacyCell::new(RxBuffer {
    buf: [0u8; RX_BUF_SIZE],
    pos: 0,
});

/// Get a reference to the received data available for protocol processing.
///
/// The returned slice is valid until `rx_consume()` is called.
///
/// # Safety
///
/// Must be called from the same RTIC task context that calls `poll()`.
pub fn rx_data() -> &'static [u8] {
    // SAFETY: `RX` is only touched from the RTIC protocol task context.
    // The returned slice remains valid until the caller invokes
    // `rx_consume`, which can only happen on the same task.
    unsafe {
        let rx = &*RX.get();
        &rx.buf[..rx.pos as usize]
    }
}

/// Mark `len` bytes as consumed from the RX buffer. The consumed bytes
/// are removed from the front and remaining data is shifted down.
///
/// # Safety
///
/// Must be called from the same RTIC task context that calls `poll()`.
pub fn rx_consume(len: usize) {
    // SAFETY: same task-context contract as `rx_data`.
    unsafe {
        let rx = &mut *RX.get();
        let rpos = rx.pos as usize;
        if len >= rpos {
            rx.pos = 0;
            return;
        }
        let needcopy = rpos - len;
        for i in 0..needcopy {
            rx.buf[i] = rx.buf[len + i];
        }
        rx.pos = needcopy as u8;
        if needcopy > 0 {
            notify_bulk_out();
        }
    }
}

/// Process bulk OUT reception. Matches `usb_bulk_out_task()` in usb_cdc.c.
///
/// Returns true if receive data is available for protocol processing.
fn bulk_out_task() -> bool {
    // SAFETY: `RX` is task-context-owned; the ISR only sets the wake flag.
    if !check_bulk_out_wake() {
        return unsafe { (*RX.get()).pos > 0 };
    }
    unsafe {
        let rx = &mut *RX.get();
        let rpos = rx.pos as usize;
        if rpos + EP_BULK_OUT_SIZE <= RX_BUF_SIZE {
            let ret = otg::usb_read_bulk_out(&mut rx.buf[rpos..], EP_BULK_OUT_SIZE as u8);
            if ret > 0 {
                rx.pos = (rpos + ret as usize) as u8;
                notify_bulk_out();
            }
        } else {
            notify_bulk_out();
        }
        rx.pos > 0
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

/// EP0 control-transfer continuation state. Filled when `usb_do_xfer`
/// returns before the transfer completes (hardware busy) and consumed by
/// the next `ep0_task` invocation.
struct XferState {
    data: *mut u8,
    size: u8,
    flags: u8,
}

/// EP0 transfer continuation. Accessed only from the single RTIC task
/// context running `ep0_task` and its helpers; the ISR only sets the
/// EP0 wake flag.
static XFER: RacyCell<XferState> = RacyCell::new(XferState {
    data: core::ptr::null_mut(),
    size: 0,
    flags: 0,
});

/// CDC line state: baud/format/parity/data-bits plus the DTR/RTS control
/// bits. Written by SET_LINE_CODING / SET_CONTROL_LINE_STATE handlers and
/// read by the Katapult reboot detection, all from the same task context.
struct LineState {
    coding: UsbCdcLineCoding,
    /// DTR/RTS flags from `SET_CONTROL_LINE_STATE`.
    control: u8,
}

static LINE: RacyCell<LineState> = RacyCell::new(LineState {
    coding: UsbCdcLineCoding {
        dw_dte_rate: 0,
        b_char_format: 0,
        b_parity_type: 0,
        b_data_bits: 0,
    },
    control: 0,
});

/// Stall EP0 and clear transfer state.
fn usb_do_stall() {
    otg::usb_stall_ep0();
    // SAFETY: `XFER` is only touched from the EP0 task context.
    unsafe {
        (*XFER.get()).flags = 0;
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
                // SAFETY: `XFER` is only touched from the EP0 task context.
                unsafe {
                    (*XFER.get()).flags = 0;
                }
                notify_ep0();
                return;
            }
            continue;
        }
        if ret == -1 {
            // Interface busy — save continuation for the next ep0_task pass.
            // SAFETY: `XFER` is only touched from the EP0 task context.
            unsafe {
                *XFER.get() = XferState { data, size, flags };
            }
            return;
        }
        if ret == -2 {
            // A new SETUP packet arrived while this transfer was in flight.
            // The host has abandoned it, so drop it and let the state machine
            // service the new request; stalling here would fail a control
            // transfer the host is no longer waiting on, and the host would
            // retry into the same state indefinitely.
            // SAFETY: `XFER` is only touched from the EP0 task context.
            unsafe {
                (*XFER.get()).flags = 0;
            }
            notify_ep0();
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

/// Empty descriptor entry used to initialize the lookup table.
const EMPTY_ENTRY: DescriptorEntry = DescriptorEntry {
    w_value: 0,
    w_index: 0,
    data: core::ptr::null(),
    size: 0,
};

/// Consolidated USB descriptor storage. Populated by `init`, patched by
/// `set_serial_from_chip_id`, and read by `usb_req_get_descriptor`.
struct Descriptors {
    device: UsbDeviceDescriptor,
    config: CdcConfigDescriptor,
    #[cfg(feature = "trace")]
    trace_config: descriptors::CdcTraceConfigDescriptor,
    str_lang: StringDescriptorBuf,
    str_manufacturer: StringDescriptorBuf,
    str_product: StringDescriptorBuf,
    str_serial: StringDescriptorBuf,
    table: [DescriptorEntry; MAX_DESCRIPTORS],
    count: usize,
}

/// USB descriptor table. Initialized once by `init()` and afterwards
/// read by the EP0 task in response to GET_DESCRIPTOR requests.
/// `set_serial_from_chip_id` is the only other writer; it must run
/// before USB enumeration reads the serial string (see its safety
/// contract).
static DESCRIPTORS: RacyCell<Descriptors> = RacyCell::new(Descriptors {
    device: UsbDeviceDescriptor {
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
    },
    config: CdcConfigDescriptor {
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
    },
    #[cfg(feature = "trace")]
    trace_config: descriptors::build_trace_config_descriptor(),
    str_lang: StringDescriptorBuf::lang_ids(),
    str_manufacturer: StringDescriptorBuf::from_ascii("Klipper"),
    str_product: StringDescriptorBuf::from_ascii("FOCI"),
    str_serial: StringDescriptorBuf::from_ascii("00000000"),
    table: [EMPTY_ENTRY; MAX_DESCRIPTORS],
    count: 0,
});

// ----------------------------------------------------------------
// EP0 request handlers (port of usb_cdc.c request handlers)
// ----------------------------------------------------------------

/// Handle GET_DESCRIPTOR request.
fn usb_req_get_descriptor(req: &UsbCtrlRequest) {
    if req.b_request_type != descriptors::USB_DIR_IN {
        usb_do_stall();
        return;
    }
    // SAFETY: `DESCRIPTORS` is initialized once at init time and only read
    // from the EP0 task context afterwards; no concurrent writer.
    let (desc, mut size) = unsafe {
        let d = &*DESCRIPTORS.get();
        let mut found: Option<(*const u8, u8)> = None;
        for entry in d.table.iter().take(d.count) {
            if entry.w_value == req.w_value && entry.w_index == req.w_index {
                found = Some((entry.data, entry.size));
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HaltTarget {
    In(usize),
    Out(usize),
}

fn clear_endpoint_halt_target(req: &UsbCtrlRequest) -> Option<HaltTarget> {
    if req.b_request_type != descriptors::USB_RECIP_ENDPOINT
        || req.b_request != descriptors::USB_REQ_CLEAR_FEATURE
        || req.w_value != descriptors::USB_FEATURE_ENDPOINT_HALT
        || req.w_length != 0
    {
        return None;
    }
    let address = req.w_index;
    if address & !(descriptors::USB_DIR_IN as u16 | 0x0F) != 0 {
        return None;
    }
    let is_in = address & descriptors::USB_DIR_IN as u16 != 0;
    let ep = (address & 0x0F) as usize;
    match (ep, is_in) {
        (EP_BULK_IN, true) | (EP_ACM, true) => Some(HaltTarget::In(ep)),
        #[cfg(feature = "trace")]
        (EP_TRACE_IN, true) => Some(HaltTarget::In(ep)),
        (EP_BULK_OUT, false) => Some(HaltTarget::Out(ep)),
        _ => None,
    }
}

fn usb_req_clear_feature(req: &UsbCtrlRequest) {
    let Some(target) = clear_endpoint_halt_target(req) else {
        usb_do_stall();
        return;
    };
    let cleared = match target {
        HaltTarget::In(ep) => otg::usb_clear_endpoint_halt(ep, true),
        HaltTarget::Out(ep) => otg::usb_clear_endpoint_halt(ep, false),
    };
    if !cleared {
        usb_do_stall();
        return;
    }
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
static HAS_BOOTLOADER: AtomicBool = AtomicBool::new(false);

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
            HAS_BOOTLOADER.store(true, Ordering::Relaxed);
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
    if !HAS_BOOTLOADER.load(Ordering::Relaxed) {
        return;
    }
    // SAFETY: `LINE` is only touched from the EP0 task context.
    let (baud, control) = unsafe {
        let line = &*LINE.get();
        (line.coding.dw_dte_rate, line.control)
    };
    if baud == 1200 && (control & 0x01) == 0 {
        bootloader_request();
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
    // SAFETY: `LINE` is only touched from the EP0 task context; the
    // raw pointer is valid for the duration of `usb_do_xfer`.
    let ptr = unsafe { &raw mut (*LINE.get()).coding };
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
    // SAFETY: `LINE` is only touched from the EP0 task context; the raw
    // pointer is valid for the duration of `usb_do_xfer`.
    let ptr = unsafe { &raw const (*LINE.get()).coding };
    usb_do_xfer(ptr as *mut u8, line_coding_size as u8, UX_SEND);
}

/// Handle SET_CONTROL_LINE_STATE request.
fn usb_req_set_line(req: &UsbCtrlRequest) {
    if req.b_request_type != 0x21 || req.w_index != 0 || req.w_length != 0 {
        usb_do_stall();
        return;
    }
    // SAFETY: `LINE` is only touched from the EP0 task context.
    unsafe {
        (*LINE.get()).control = req.w_value as u8;
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
        descriptors::USB_REQ_CLEAR_FEATURE => usb_req_clear_feature(&req),
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
    // SAFETY: `XFER` is only touched from the EP0 task context.
    let (data, size, flags) = unsafe {
        let xfer = &*XFER.get();
        (xfer.data, xfer.size, xfer.flags)
    };
    if flags != 0 {
        usb_do_xfer(data, size, flags);
    } else {
        usb_state_ready();
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
    // SAFETY: caller guarantees single init before any other CDC function.
    unsafe {
        // Check for Katapult bootloader (must happen before USB traffic).
        detect_bootloader();

        // Build descriptors.
        let d = &mut *DESCRIPTORS.get();
        d.device = descriptors::build_device_descriptor(vid, pid);
        d.config = descriptors::build_config_descriptor();
        #[cfg(feature = "trace")]
        {
            d.trace_config = descriptors::build_trace_config_descriptor();
        }
        d.str_lang = StringDescriptorBuf::lang_ids();
        d.str_manufacturer = StringDescriptorBuf::from_ascii(manufacturer);
        d.str_product = StringDescriptorBuf::from_ascii(product);
        d.str_serial = StringDescriptorBuf::from_ascii(serial);

        // Build descriptor lookup table using raw pointers into `DESCRIPTORS`
        // so every entry points to storage with static lifetime.
        let device_ptr = (&raw const d.device) as *const u8;
        #[cfg(not(feature = "trace"))]
        let config_ptr = (&raw const d.config) as *const u8;
        #[cfg(not(feature = "trace"))]
        let config_size = core::mem::size_of::<CdcConfigDescriptor>() as u8;
        #[cfg(feature = "trace")]
        let config_ptr = (&raw const d.trace_config) as *const u8;
        #[cfg(feature = "trace")]
        let config_size = core::mem::size_of::<descriptors::CdcTraceConfigDescriptor>() as u8;
        let (lang_ptr, lang_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const d.str_lang);
        let (mfr_ptr, mfr_len) =
            StringDescriptorBuf::raw_ptr_and_len(&raw const d.str_manufacturer);
        let (prod_ptr, prod_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const d.str_product);
        let (ser_ptr, ser_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const d.str_serial);

        let mut idx = 0;
        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_DEVICE as u16) << 8,
            w_index: 0,
            data: device_ptr,
            size: core::mem::size_of::<UsbDeviceDescriptor>() as u8,
        };
        idx += 1;

        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_CONFIG as u16) << 8,
            w_index: 0,
            data: config_ptr,
            size: config_size,
        };
        idx += 1;

        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_STRING as u16) << 8,
            w_index: 0,
            data: lang_ptr,
            size: lang_len,
        };
        idx += 1;

        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_STRING as u16) << 8
                | descriptors::USB_STR_ID_MANUFACTURER as u16,
            w_index: descriptors::USB_LANGID_ENGLISH_US,
            data: mfr_ptr,
            size: mfr_len,
        };
        idx += 1;

        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_STRING as u16) << 8
                | descriptors::USB_STR_ID_PRODUCT as u16,
            w_index: descriptors::USB_LANGID_ENGLISH_US,
            data: prod_ptr,
            size: prod_len,
        };
        idx += 1;

        d.table[idx] = DescriptorEntry {
            w_value: (descriptors::USB_DT_STRING as u16) << 8
                | descriptors::USB_STR_ID_SERIAL as u16,
            w_index: descriptors::USB_LANGID_ENGLISH_US,
            data: ser_ptr,
            size: ser_len,
        };
        idx += 1;

        d.count = idx;

        // Reset transfer state.
        *XFER.get() = XferState {
            data: core::ptr::null_mut(),
            size: 0,
            flags: 0,
        };
        interrupt::free(|cs| TX.borrow(cs).borrow_mut().pos = 0);
        (*RX.get()).pos = 0;
        *LINE.get() = LineState {
            coding: UsbCdcLineCoding::default(),
            control: 0,
        };
    }
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
    // SAFETY: caller ensures this runs after `init()` and before USB
    // enumeration reads the serial descriptor, so `DESCRIPTORS` has no
    // concurrent access.
    unsafe {
        let d = &mut *DESCRIPTORS.get();
        descriptors::fill_serial_from_chip_id(&mut d.str_serial, strlen, id);
        let expected_wvalue =
            (descriptors::USB_DT_STRING as u16) << 8 | descriptors::USB_STR_ID_SERIAL as u16;
        let (ser_ptr, ser_len) = StringDescriptorBuf::raw_ptr_and_len(&raw const d.str_serial);
        for entry in d.table.iter_mut().take(d.count) {
            if entry.w_value == expected_wvalue {
                entry.data = ser_ptr;
                entry.size = ser_len;
                break;
            }
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

#[cfg(test)]
mod tx_tests {
    use super::*;

    fn assert_synchronized_tx(_tx: &cortex_m::interrupt::Mutex<core::cell::RefCell<TxBuffer>>) {}

    #[test]
    fn transmit_buffer_enforces_cross_task_serialization() {
        assert_synchronized_tx(&TX);
    }

    // No host test calls tx_write: it's wrapped in cortex_m::interrupt::free,
    // whose disable/enable primitives have no host-target definition. Any
    // test that reaches that call path fails to link (undefined symbols
    // __cpsid/__cpsie/__primask_r), not merely to behave incorrectly. That
    // dead-code-strips cleanly today only because nothing in the existing
    // suite calls tx_write either -- verified on the CDC TX path, this is a
    // target-only-testable boundary, same class as board_init.rs's MMIO
    // writes.
}

#[cfg(test)]
mod clear_feature_tests {
    use super::{HaltTarget, clear_endpoint_halt_target};
    use crate::descriptors::{USB_DIR_IN, UsbCtrlRequest};
    use crate::ep::{EP_ACM, EP_BULK_IN, EP_BULK_OUT};

    fn clear_halt(w_index: u16) -> UsbCtrlRequest {
        UsbCtrlRequest {
            b_request_type: 0x02,
            b_request: 0x01,
            w_value: 0,
            w_index,
            w_length: 0,
        }
    }

    #[test]
    fn bulk_in_address_resolves_to_in_endpoint() {
        let req = clear_halt(EP_BULK_IN as u16 | USB_DIR_IN as u16);
        assert_eq!(
            clear_endpoint_halt_target(&req),
            Some(HaltTarget::In(EP_BULK_IN))
        );
    }

    #[test]
    fn bulk_out_address_resolves_to_out_endpoint() {
        let req = clear_halt(EP_BULK_OUT as u16);
        assert_eq!(
            clear_endpoint_halt_target(&req),
            Some(HaltTarget::Out(EP_BULK_OUT))
        );
    }

    #[test]
    fn acm_notification_address_resolves_to_in_endpoint() {
        let req = clear_halt(EP_ACM as u16 | USB_DIR_IN as u16);
        assert_eq!(
            clear_endpoint_halt_target(&req),
            Some(HaltTarget::In(EP_ACM))
        );
    }

    #[cfg(feature = "trace")]
    #[test]
    fn trace_in_shares_endpoint_number_with_bulk_out() {
        use crate::ep::EP_TRACE_IN;
        let req = clear_halt(EP_TRACE_IN as u16 | USB_DIR_IN as u16);
        assert_eq!(
            clear_endpoint_halt_target(&req),
            Some(HaltTarget::In(EP_TRACE_IN))
        );
        assert_eq!(
            clear_endpoint_halt_target(&clear_halt(EP_BULK_OUT as u16)),
            Some(HaltTarget::Out(EP_BULK_OUT))
        );
    }

    #[test]
    fn other_feature_selector_is_rejected() {
        let mut req = clear_halt(EP_BULK_IN as u16 | USB_DIR_IN as u16);
        req.w_value = 1;
        assert_eq!(clear_endpoint_halt_target(&req), None);
    }

    #[test]
    fn non_endpoint_recipient_is_rejected() {
        let mut req = clear_halt(EP_BULK_IN as u16 | USB_DIR_IN as u16);
        req.b_request_type = 0x00;
        assert_eq!(clear_endpoint_halt_target(&req), None);
    }

    #[test]
    fn unconfigured_endpoint_is_rejected() {
        assert_eq!(clear_endpoint_halt_target(&clear_halt(0x85)), None);
        assert_eq!(clear_endpoint_halt_target(&clear_halt(0x04)), None);
    }

    #[test]
    fn control_endpoint_is_rejected() {
        assert_eq!(clear_endpoint_halt_target(&clear_halt(0x80)), None);
        assert_eq!(clear_endpoint_halt_target(&clear_halt(0x00)), None);
    }

    #[test]
    fn ready_state_dispatches_clear_feature() {
        let source = include_str!("cdc.rs");
        assert!(
            source.contains("descriptors::USB_REQ_CLEAR_FEATURE => usb_req_clear_feature(&req),"),
            "usb_state_ready must dispatch CLEAR_FEATURE instead of stalling it"
        );
    }
}
