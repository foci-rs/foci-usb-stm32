// Port of Klipper's src/stm32/usbotg.c to Rust.
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Morton Jonuschat
//
// This file may be distributed under the terms of the GNU GPLv3 license.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicUsize, Ordering};

use crate::cdc;
use crate::ep::{EP_ACM, EP_ACM_SIZE, EP_BULK_IN, EP_BULK_IN_SIZE, EP_BULK_OUT, EP_BULK_OUT_SIZE};
#[cfg(feature = "trace")]
use crate::ep::{EP_TRACE_IN, EP_TRACE_IN_SIZE};
use crate::sync::RacyCell;

// Global registers (base + 0x000)
const GOTGCTL: usize = 0x000;
const GRSTCTL: usize = 0x010;
const GINTSTS: usize = 0x014;
const GINTMSK: usize = 0x018;
const GRXSTSR: usize = 0x01C;
const GRXSTSP: usize = 0x020;
const GRXFSIZ: usize = 0x024;
const DIEPTXF0: usize = 0x028; // DIEPTXF0_HNPTXFSIZ
const GCCFG: usize = 0x038;
const GAHBCFG: usize = 0x008;
const GUSBCFG: usize = 0x00C;

// DIEPTXF registers (for endpoints 1..N)
const fn dieptxf(n: usize) -> usize {
    0x104 + (n - 1) * 4
}

// Device registers (base + 0x800)
const DCFG: usize = 0x800;
const DCTL: usize = 0x804;
const DIEPMSK: usize = 0x810;
const DAINTMSK: usize = 0x81C;
const DAINT: usize = 0x818;

// IN endpoint registers (base + 0x900 + ep * 0x20)
const fn diepctl(ep: usize) -> usize {
    0x900 + ep * 0x20
}
const fn diepint(ep: usize) -> usize {
    0x908 + ep * 0x20
}
const fn dieptsiz(ep: usize) -> usize {
    0x910 + ep * 0x20
}

// OUT endpoint registers (base + 0xB00 + ep * 0x20)
const fn doepctl(ep: usize) -> usize {
    0xB00 + ep * 0x20
}
const fn doeptsiz(ep: usize) -> usize {
    0xB10 + ep * 0x20
}
const fn doepint(ep: usize) -> usize {
    0xB08 + ep * 0x20
}

// FIFO access (base + 0x1000 + ep * 0x1000)
const fn fifo_addr(ep: usize) -> usize {
    0x1000 + ep * 0x1000
}

const GUSBCFG_FDMOD: u32 = 1 << 30;
const GUSBCFG_PHYSEL: u32 = 1 << 6;
const GUSBCFG_TRDT_POS: u32 = 10;

const GRSTCTL_AHBIDL: u32 = 1 << 31;
const GRSTCTL_TXFFLSH: u32 = 1 << 5;
const GRSTCTL_TXFNUM_POS: u32 = 6;

const GINTSTS_RXFLVL: u32 = 1 << 4;
const GINTMSK_RXFLVLM: u32 = 1 << 4;
const GINTSTS_IEPINT: u32 = 1 << 18;
const GINTMSK_IEPINT: u32 = 1 << 18;

const GAHBCFG_GINT: u32 = 1 << 0;

const GCCFG_NOVBUSSENS: u32 = 1 << 21;
const GCCFG_PWRDWN: u32 = 1 << 16;

const GOTGCTL_BVALOEN: u32 = 1 << 6;
const GOTGCTL_BVALOVAL: u32 = 1 << 7;

const DCFG_DSPD_POS: u32 = 0;
const DCFG_DAD_POS: u32 = 4;
const DCFG_DAD_MSK: u32 = 0x7F << DCFG_DAD_POS;

const DEPCTL_EPENA: u32 = 1 << 31;
const DEPCTL_EPDIS: u32 = 1 << 30;
const DEPCTL_SNAK: u32 = 1 << 27;
const DEPCTL_CNAK: u32 = 1 << 26;
const DEPCTL_STALL: u32 = 1 << 21;
const DEPCTL_USBAEP: u32 = 1 << 15;
const DEPCTL_NAKSTS: u32 = 1 << 17;
const DEPCTL_SD0PID: u32 = 1 << 28;
const DIEPCTL_TXFNUM_POS: u32 = 22;
const DEPCTL_EPTYP_POS: u32 = 18;
const DEPCTL_MPSIZ_POS: u32 = 0;

const DEPTSIZ_PKTCNT_POS: u32 = 19;
const DOEPTSIZ_STUPCNT_POS: u32 = 29;

const DIEPINT_XFRC: u32 = 1 << 0;

const DOEPINT_STUP: u32 = 1 << 3;

const DIEPMSK_XFRCM: u32 = 1 << 0;

const GRXSTSP_EPNUM_MSK: u32 = 0x0F;
const GRXSTSP_BCNT_POS: u32 = 4;
const GRXSTSP_BCNT_MSK: u32 = 0x7FF << GRXSTSP_BCNT_POS;
const GRXSTSP_PKTSTS_POS: u32 = 17;
const GRXSTSP_PKTSTS_MSK: u32 = 0x0F << GRXSTSP_PKTSTS_POS;

const TX0FSA_POS: u32 = 0;
const TX0FD_POS: u32 = 16;

// STM32F407 OTG FS has 320 32-bit FIFO words total.
const OTG_FS_FIFO_WORDS: u32 = 320;
const NUM_EP: u32 = 1;
const NUM_OUT_EP: u32 = 1;
const TX_FIFO_WORDS: u32 = 0x10;
#[cfg(feature = "trace")]
const TRACE_TX_FIFO_WORDS: u32 = 0x20;
// MPSIZ encoding: 0=64, 1=32, 2=16, 3=8 bytes
const DEPCTL_MPSIZ_16_BYTES: u32 = 2;

/// Base address of the USB OTG peripheral. Set once during init.
static USB_BASE: AtomicUsize = AtomicUsize::new(0);

/// IRQ number for the USB OTG peripheral. Set once during init.
static USB_IRQ: AtomicU16 = AtomicU16::new(0);

/// Whether double-buffer TX is enabled. Set once during init.
static DOUBLE_BUFFER_TX: AtomicBool = AtomicBool::new(false);

/// Wake flag: set by ISR, polled by RTIC task.
static WAKE_FLAG: AtomicBool = AtomicBool::new(false);

/// TX double-buffer.
#[derive(Clone, Copy)]
struct TxBuf {
    len: u32,
    data: [u32; EP_BULK_IN_SIZE / 4],
}

/// USB bulk IN double-buffer state. Accessed from task context and ISR;
/// all accesses are serialized either by `usb_irq_disable`/`_enable` windows
/// in task code or by the single ISR context owning the handler.
static TX_BUF: RacyCell<TxBuf> = RacyCell::new(TxBuf {
    len: 0,
    data: [0; EP_BULK_IN_SIZE / 4],
});

#[inline(always)]
fn readl(addr: usize) -> u32 {
    // SAFETY: addr is a memory-mapped register address set during init
    // from a known-valid peripheral base. Protected by usb_irq_disable/enable.
    unsafe { read_volatile(addr as *const u32) }
}

#[inline(always)]
fn writel(addr: usize, val: u32) {
    // SAFETY: addr is a memory-mapped register address set during init
    // from a known-valid peripheral base. Protected by usb_irq_disable/enable.
    unsafe { write_volatile(addr as *mut u32, val) }
}

#[inline(always)]
fn reg(offset: usize) -> usize {
    USB_BASE.load(Ordering::Relaxed) + offset
}

/// Disable the USB interrupt (does not disable all interrupts).
pub fn usb_irq_disable() {
    // `cortex_m::peripheral::NVIC::mask()` is safe to call with any valid
    // IRQ number. `USB_IRQ` is set once during init and never changed.
    cortex_m::peripheral::NVIC::mask(IrqNr(USB_IRQ.load(Ordering::Relaxed)));
    // Writing NVIC_ICER is posted; without these barriers the handler can
    // still take the core after this returns, leaving the caller's critical
    // section open. CMSIS's NVIC_DisableIRQ, which the reference driver uses,
    // ends with the same DSB/ISB pair.
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Enable the USB interrupt.
pub fn usb_irq_enable() {
    // SAFETY: enabling an interrupt with a valid IRQ number is sound; this
    // must still be an `unsafe` call because `NVIC::unmask` can enable
    // interrupts that break mask-based critical sections elsewhere.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(IrqNr(USB_IRQ.load(Ordering::Relaxed)));
    }
}

/// Wrapper to satisfy cortex_m::interrupt::InterruptNumber trait.
#[derive(Clone, Copy)]
struct IrqNr(u16);

unsafe impl cortex_m::interrupt::InterruptNumber for IrqNr {
    fn number(self) -> u16 {
        self.0
    }
}

/// Signal the RTIC task to wake up and poll USB.
pub fn notify_wake() {
    WAKE_FLAG.store(true, Ordering::Release);
}

/// Check and clear the wake flag. Returns true if USB needs polling.
pub fn check_wake() -> bool {
    WAKE_FLAG.swap(false, Ordering::AcqRel)
}

/// Setup the USB FIFOs.
fn fifo_configure() {
    let sz: u32 = (4 * NUM_EP + 6) + 4 * ((EP_BULK_OUT_SIZE as u32 / 4) + 1) + (2 * NUM_OUT_EP);
    writel(reg(GRXFSIZ), sz);

    let mut fpos = sz;
    let ep_size = TX_FIFO_WORDS;

    writel(reg(DIEPTXF0), (fpos << TX0FSA_POS) | (ep_size << TX0FD_POS));
    fpos += ep_size;

    writel(
        reg(dieptxf(EP_ACM)),
        (fpos << TX0FSA_POS) | (ep_size << TX0FD_POS),
    );
    fpos += ep_size;

    writel(
        reg(dieptxf(EP_BULK_IN)),
        (fpos << TX0FSA_POS) | (ep_size << TX0FD_POS),
    );
    fpos += ep_size;

    #[cfg(feature = "trace")]
    {
        let trace_ep_size = TRACE_TX_FIFO_WORDS;
        writel(
            reg(dieptxf(EP_TRACE_IN)),
            (fpos << TX0FSA_POS) | (trace_ep_size << TX0FD_POS),
        );
        fpos += trace_ep_size;
    }

    debug_assert!(fpos <= OTG_FS_FIFO_WORDS);
}

/// Why an EP0 or bulk endpoint operation moved no data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpError {
    /// The hardware has nothing to read or no room to write; retry when the endpoint wakes the
    /// task.
    NotReady,
    /// A SETUP packet arrived, so the host abandoned the control transfer in flight.
    Interrupted,
}

fn classify_ep0_read(grx: u32) -> Result<(), EpError> {
    if grx == 0 {
        return Err(EpError::NotReady);
    }
    let pktsts = (grx & GRXSTSP_PKTSTS_MSK) >> GRXSTSP_PKTSTS_POS;
    if pktsts != 2 {
        return Err(EpError::Interrupted);
    }
    Ok(())
}

fn classify_ep0_send(grx: u32, diepctl0: u32) -> Result<(), EpError> {
    if grx != 0 {
        return Err(EpError::Interrupted);
    }
    if diepctl0 & DEPCTL_EPENA != 0 {
        return Err(EpError::NotReady);
    }
    Ok(())
}

/// Write a packet to a TX FIFO.
///
/// Returns the number of bytes written (always `len`).
pub fn fifo_write_packet(ep: usize, src: &[u8]) -> i8 {
    let fifo = reg(fifo_addr(ep));
    let len = src.len() as u32;

    writel(reg(diepint(ep)), DIEPINT_XFRC);
    writel(reg(dieptsiz(ep)), len | (1 << DEPTSIZ_PKTCNT_POS));
    let ctl = readl(reg(diepctl(ep)));
    writel(reg(diepctl(ep)), ctl | DEPCTL_EPENA | DEPCTL_CNAK);

    let mut offset = 0usize;
    let mut count = len as i32;
    while count >= 4 {
        let mut word = [0u8; 4];
        word.copy_from_slice(&src[offset..offset + 4]);
        writel(fifo, u32::from_ne_bytes(word));
        count -= 4;
        offset += 4;
    }
    if count > 0 {
        let mut word = [0u8; 4];
        word[..count as usize].copy_from_slice(&src[offset..offset + count as usize]);
        writel(fifo, u32::from_ne_bytes(word));
    }

    len as i8
}

/// Write a packet to a TX FIFO from aligned u32 data.
///
///
/// Returns 0 on success, -1 if the endpoint is still busy.
fn fifo_write_packet_fast(ep: usize, src: &[u32], len: u32) -> i32 {
    let fifo = reg(fifo_addr(ep));
    let ctl = readl(reg(diepctl(ep)));
    if ctl & DEPCTL_EPENA != 0 {
        return -1;
    }
    writel(reg(diepint(ep)), DIEPINT_XFRC);
    writel(reg(dieptsiz(ep)), len | (1 << DEPTSIZ_PKTCNT_POS));
    writel(reg(diepctl(ep)), ctl | DEPCTL_EPENA | DEPCTL_CNAK);
    let words = len.div_ceil(4) as usize;
    for word in src.iter().take(words) {
        writel(fifo, *word);
    }
    0
}

/// Read a packet from the RX queue.
///
/// Returns the number of bytes transferred (capped at `max_len`).
/// If `dest` is `None`, reads and discards the packet data.
pub fn fifo_read_packet(dest: Option<&mut [u8]>, max_len: u8) -> u8 {
    let fifo = reg(fifo_addr(0));
    let grx = readl(reg(GRXSTSP));
    let bcnt = (grx & GRXSTSP_BCNT_MSK) >> GRXSTSP_BCNT_POS;
    let xfer = if bcnt > max_len as u32 {
        max_len as u32
    } else {
        bcnt
    };
    let mut count = xfer;
    let mut offset = 0usize;

    if let Some(buf) = dest {
        while count >= 4 {
            let data = readl(fifo);
            let bytes = data.to_ne_bytes();
            buf[offset..offset + 4].copy_from_slice(&bytes);
            count -= 4;
            offset += 4;
        }
        if count > 0 {
            let data = readl(fifo);
            let bytes = data.to_ne_bytes();
            buf[offset..offset + count as usize].copy_from_slice(&bytes[..count as usize]);
        }
    } else {
        while count >= 4 {
            let _ = readl(fifo);
            count -= 4;
        }
        if count > 0 {
            let _ = readl(fifo);
        }
    }

    let extra = bcnt.div_ceil(4).saturating_sub(xfer.div_ceil(4));
    for _ in 0..extra {
        let _ = readl(fifo);
    }

    xfer as u8
}

/// Re-enable packet reception on an OUT endpoint.
fn enable_rx_endpoint(ep: usize) {
    let ctl = readl(reg(doepctl(ep)));
    if ctl & DEPCTL_EPENA == 0 || ctl & DEPCTL_NAKSTS != 0 {
        let stupcnt = if ep == 0 {
            3 << DOEPTSIZ_STUPCNT_POS
        } else {
            0
        };
        writel(reg(doeptsiz(ep)), 64 | (1 << DEPTSIZ_PKTCNT_POS) | stupcnt);
        writel(reg(doepctl(ep)), ctl | DEPCTL_EPENA | DEPCTL_CNAK);
    }
}

/// Peek at the next packet in the RX queue without consuming it.
///
///
/// Returns the GRXSTSR value if a packet for `ep` is ready, or 0 if not.
fn peek_rx_queue(ep: usize) -> u32 {
    loop {
        let sts = readl(reg(GINTSTS));
        if sts & GINTSTS_RXFLVL == 0 {
            return 0;
        }
        let grx = readl(reg(GRXSTSR));
        let grx_ep = grx & GRXSTSP_EPNUM_MSK;
        let pktsts = (grx & GRXSTSP_PKTSTS_MSK) >> GRXSTSP_PKTSTS_POS;
        if (grx_ep == 0 || grx_ep == EP_BULK_OUT as u32)
            && (pktsts == 2 || pktsts == 4 || pktsts == 6)
        {
            if grx_ep != ep as u32 {
                return 0;
            }
            return grx;
        }
        if (grx_ep != 0 && grx_ep != EP_BULK_OUT as u32)
            || (pktsts != 1 && pktsts != 3 && pktsts != 4)
        {
            let _ = readl(reg(GRXSTSP));
            continue;
        }
        fifo_read_packet(None, 0);
    }
}

/// Read data from the bulk OUT endpoint.
/// Returns the number of bytes read, or `EpError::NotReady` if no data is available.
pub fn usb_read_bulk_out(data: &mut [u8], max_len: u8) -> Result<u8, EpError> {
    usb_irq_disable();
    let grx = peek_rx_queue(EP_BULK_OUT);
    if grx == 0 {
        let mask = readl(reg(GINTMSK));
        writel(reg(GINTMSK), mask | GINTMSK_RXFLVLM);
        usb_irq_enable();
        return Err(EpError::NotReady);
    }
    let ret = fifo_read_packet(Some(data), max_len);
    enable_rx_endpoint(EP_BULK_OUT);
    usb_irq_enable();
    Ok(ret)
}

/// Send data on the bulk IN endpoint.
/// Returns the number of bytes sent, or `EpError::NotReady` if the endpoint is busy.
pub fn usb_send_bulk_in(data: &[u8]) -> Result<u8, EpError> {
    let len = data.len();
    usb_irq_disable();
    let ctl = readl(reg(diepctl(EP_BULK_IN)));
    if ctl & DEPCTL_USBAEP == 0 {
        usb_irq_enable();
        return Ok(len as u8);
    }

    let double_buf = DOUBLE_BUFFER_TX.load(Ordering::Relaxed);
    // SAFETY: TX_BUF is accessed inside a `usb_irq_disable`/_enable window
    // and the ISR cannot preempt. No other context observes `TX_BUF` here.
    let dbuf_busy = double_buf && unsafe { (*TX_BUF.get()).len } != 0;

    if ctl & DEPCTL_EPENA != 0 || dbuf_busy {
        // SAFETY: same access rationale as above for `TX_BUF.len`.
        if !double_buf || unsafe { (*TX_BUF.get()).len } != 0 || len == 0 {
            let msk = readl(reg(DAINTMSK));
            writel(reg(DAINTMSK), msk | (1 << EP_BULK_IN));
            usb_irq_enable();
            return Err(EpError::NotReady);
        }
        let len = if len > EP_BULK_IN_SIZE {
            EP_BULK_IN_SIZE
        } else {
            len
        };
        let blocks = (len as u32).div_ceil(4) as usize;
        // SAFETY: TX_BUF is protected by the surrounding
        // `usb_irq_disable`/_enable window. The ISR writes only to `len`
        // (via `irq_handler`), which cannot run while masked.
        unsafe {
            let tx = &mut *TX_BUF.get();
            tx.data[blocks - 1] = 0;
            core::ptr::copy_nonoverlapping(data.as_ptr(), tx.data.as_mut_ptr() as *mut u8, len);
            tx.len = len as u32;
        }
        let msk = readl(reg(DAINTMSK));
        writel(reg(DAINTMSK), msk | (1 << EP_BULK_IN));
        usb_irq_enable();
        return Ok(len as u8);
    }
    let ret = fifo_write_packet(EP_BULK_IN, &data[..len]);
    usb_irq_enable();
    Ok(ret.unsigned_abs())
}

/// Send data on the trace bulk IN endpoint.
///
/// Returns number of bytes sent, or -1 if busy. The caller is responsible for
/// splitting larger frames into full-speed packets.
#[cfg(feature = "trace")]
pub fn usb_send_trace_in(data: &[u8]) -> i8 {
    let len = data.len().min(EP_TRACE_IN_SIZE);
    usb_irq_disable();
    let ctl = readl(reg(diepctl(EP_TRACE_IN)));
    if ctl & DEPCTL_USBAEP == 0 {
        // Controller not enabled. Report no accepted bytes so callers can
        // count endpoint-not-ready pressure instead of silently losing frames.
        usb_irq_enable();
        return 0;
    }
    if ctl & DEPCTL_EPENA != 0 {
        let msk = readl(reg(DAINTMSK));
        writel(reg(DAINTMSK), msk | (1 << EP_TRACE_IN));
        usb_irq_enable();
        return -1;
    }
    let ret = fifo_write_packet(EP_TRACE_IN, &data[..len]);
    if trace_packet_needs_completion_wake(ret) {
        let msk = readl(reg(DAINTMSK));
        writel(reg(DAINTMSK), msk | (1 << EP_TRACE_IN));
    }
    usb_irq_enable();
    ret
}

#[cfg(feature = "trace")]
fn trace_packet_needs_completion_wake(write_result: i8) -> bool {
    write_result > 0
}

/// Read data from EP0 (non-setup data phase).
/// Returns the number of bytes read, `EpError::NotReady` if no data is queued, or
/// `EpError::Interrupted` if the next packet is not a data packet.
pub fn usb_read_ep0(data: &mut [u8], max_len: u8) -> Result<u8, EpError> {
    usb_irq_disable();
    let grx = peek_rx_queue(0);
    if let Err(err) = classify_ep0_read(grx) {
        if err == EpError::NotReady {
            let mask = readl(reg(GINTMSK));
            writel(reg(GINTMSK), mask | GINTMSK_RXFLVLM);
        }
        usb_irq_enable();
        return Err(err);
    }
    let ret = fifo_read_packet(Some(data), max_len);
    enable_rx_endpoint(0);
    usb_irq_enable();
    Ok(ret)
}

const FLUSH_POLL_LIMIT: u32 = 100_000;

fn poll_until(limit: u32, mut done: impl FnMut() -> bool) -> bool {
    (0..limit).any(|_| done())
}

/// Disable and flush a pending TX packet on EP0 IN, if one is in flight.
/// Leaves the endpoint disabled; the next `fifo_write_packet` re-enables it.
fn flush_ep0_in() {
    let ctl = readl(reg(diepctl(0)));
    if ctl & DEPCTL_EPENA == 0 {
        return;
    }
    writel(reg(diepctl(0)), ctl | DEPCTL_EPDIS | DEPCTL_SNAK);
    if !poll_until(FLUSH_POLL_LIMIT, || {
        readl(reg(diepctl(0))) & DEPCTL_EPENA == 0
    }) {
        // TXFFLSH must not be written while the core may still read the FIFO.
        return;
    }
    writel(reg(GRSTCTL), GRSTCTL_TXFFLSH);
    poll_until(FLUSH_POLL_LIMIT, || {
        readl(reg(GRSTCTL)) & GRSTCTL_TXFFLSH == 0
    });
}

/// Read a SETUP packet from EP0.
/// Returns the size of the setup packet, or -1 if not ready.
pub fn usb_read_ep0_setup(data: &mut [u8], max_len: u8) -> i8 {
    /// Persistent SETUP staging buffer owned by `usb_read_ep0_setup`.
    /// Only touched inside a `usb_irq_disable`/_enable window.
    static SETUP_BUF: RacyCell<[u8; 8]> = RacyCell::new([0u8; 8]);

    usb_irq_disable();
    loop {
        let grx = peek_rx_queue(0);
        if grx == 0 {
            let mask = readl(reg(GINTMSK));
            writel(reg(GINTMSK), mask | GINTMSK_RXFLVLM);
            usb_irq_enable();
            return -1;
        }
        let pktsts = (grx & GRXSTSP_PKTSTS_MSK) >> GRXSTSP_PKTSTS_POS;
        if pktsts == 6 {
            // SAFETY: SETUP_BUF is only accessed from this function,
            // which is protected by usb_irq_disable/enable.
            unsafe {
                fifo_read_packet(Some(&mut *SETUP_BUF.get()), 8);
            }
        } else {
            fifo_read_packet(None, 0);
        }
        if pktsts == 4 {
            break;
        }
    }

    flush_ep0_in();

    enable_rx_endpoint(0);
    writel(reg(doepint(0)), DOEPINT_STUP);
    usb_irq_enable();

    // SAFETY: SETUP_BUF was just filled above under irq-disabled protection.
    let copy_len = max_len.min(8) as usize;
    unsafe {
        core::ptr::copy_nonoverlapping((*SETUP_BUF.get()).as_ptr(), data.as_mut_ptr(), copy_len);
    }
    max_len as i8
}

/// Send data on EP0 (control IN).
/// Returns the number of bytes sent, `EpError::NotReady` if the IN endpoint is busy, or
/// `EpError::Interrupted` if a packet is waiting in the RX queue.
pub fn usb_send_ep0(data: &[u8]) -> Result<u8, EpError> {
    usb_irq_disable();
    let grx = peek_rx_queue(0);
    let ctl = readl(reg(diepctl(0)));
    if let Err(err) = classify_ep0_send(grx, ctl) {
        if err == EpError::NotReady {
            let mask = readl(reg(GINTMSK));
            writel(reg(GINTMSK), mask | GINTMSK_RXFLVLM);
            let msk = readl(reg(DAINTMSK));
            writel(reg(DAINTMSK), msk | (1 << 0));
        }
        usb_irq_enable();
        return Err(err);
    }
    let ret = fifo_write_packet(0, data);
    usb_irq_enable();
    Ok(ret.unsigned_abs())
}

/// Set the USB stall condition on EP0.
pub fn usb_stall_ep0() {
    usb_irq_disable();
    let ctl = readl(reg(diepctl(0)));
    writel(reg(diepctl(0)), ctl | DEPCTL_STALL);
    cdc::notify_ep0();
    usb_irq_enable();
}

fn halt_clear_control_value(ctl: u32) -> Option<u32> {
    if ctl & DEPCTL_USBAEP == 0 {
        return None;
    }
    Some((ctl & !DEPCTL_STALL) | DEPCTL_SD0PID)
}

/// Clear a halted non-control endpoint on behalf of
/// `CLEAR_FEATURE(ENDPOINT_HALT)`. Also resets the data toggle to DATA0,
/// which USB 2.0 section 9.4.5 requires alongside the halt clear.
///
/// Returns `false`, writing nothing, when the endpoint is not active in the
/// current configuration; the request is only valid in the Configured state.
pub fn usb_clear_endpoint_halt(ep: usize, is_in: bool) -> bool {
    usb_irq_disable();
    let addr = if is_in {
        reg(diepctl(ep))
    } else {
        reg(doepctl(ep))
    };
    let cleared = match halt_clear_control_value(readl(addr)) {
        Some(ctl) => {
            writel(addr, ctl);
            true
        }
        None => false,
    };
    usb_irq_enable();
    cleared
}

/// Set the USB device address.
pub fn usb_set_address(addr: u8) {
    let dcfg = readl(reg(DCFG));
    writel(
        reg(DCFG),
        (dcfg & !DCFG_DAD_MSK) | ((addr as u32) << DCFG_DAD_POS),
    );
    let _ = usb_send_ep0(&[]);
    cdc::notify_ep0();
}

/// Configure endpoints after SET_CONFIGURATION.
pub fn usb_set_configure() {
    usb_irq_disable();

    writel(
        reg(dieptsiz(EP_ACM)),
        EP_ACM_SIZE as u32 | (1 << DEPTSIZ_PKTCNT_POS),
    );
    writel(
        reg(diepctl(EP_ACM)),
        DEPCTL_SNAK
            | DEPCTL_USBAEP
            | (0x03 << DEPCTL_EPTYP_POS)
            | DEPCTL_SD0PID
            | ((EP_ACM as u32) << DIEPCTL_TXFNUM_POS)
            | ((EP_ACM_SIZE as u32) << DEPCTL_MPSIZ_POS),
    );

    writel(reg(doeptsiz(EP_BULK_OUT)), 64 | (1 << DEPTSIZ_PKTCNT_POS));
    writel(
        reg(doepctl(EP_BULK_OUT)),
        DEPCTL_CNAK
            | DEPCTL_USBAEP
            | DEPCTL_EPENA
            | (0x02 << DEPCTL_EPTYP_POS)
            | DEPCTL_SD0PID
            | ((EP_BULK_OUT_SIZE as u32) << DEPCTL_MPSIZ_POS),
    );

    writel(
        reg(dieptsiz(EP_BULK_IN)),
        EP_BULK_IN_SIZE as u32 | (1 << DEPTSIZ_PKTCNT_POS),
    );
    writel(
        reg(diepctl(EP_BULK_IN)),
        DEPCTL_SNAK
            | DEPCTL_EPDIS
            | DEPCTL_USBAEP
            | (0x02 << DEPCTL_EPTYP_POS)
            | DEPCTL_SD0PID
            | ((EP_BULK_IN as u32) << DIEPCTL_TXFNUM_POS)
            | ((EP_BULK_IN_SIZE as u32) << DEPCTL_MPSIZ_POS),
    );
    while readl(reg(diepctl(EP_BULK_IN))) & DEPCTL_EPENA != 0 {}
    writel(
        reg(GRSTCTL),
        ((EP_BULK_IN as u32) << GRSTCTL_TXFNUM_POS) | GRSTCTL_TXFFLSH,
    );
    while readl(reg(GRSTCTL)) & GRSTCTL_TXFFLSH != 0 {}

    #[cfg(feature = "trace")]
    {
        writel(
            reg(dieptsiz(EP_TRACE_IN)),
            EP_TRACE_IN_SIZE as u32 | (1 << DEPTSIZ_PKTCNT_POS),
        );
        writel(
            reg(diepctl(EP_TRACE_IN)),
            DEPCTL_SNAK
                | DEPCTL_EPDIS
                | DEPCTL_USBAEP
                | (0x02 << DEPCTL_EPTYP_POS)
                | DEPCTL_SD0PID
                | ((EP_TRACE_IN as u32) << DIEPCTL_TXFNUM_POS)
                | ((EP_TRACE_IN_SIZE as u32) << DEPCTL_MPSIZ_POS),
        );
        while readl(reg(diepctl(EP_TRACE_IN))) & DEPCTL_EPENA != 0 {}
        writel(
            reg(GRSTCTL),
            ((EP_TRACE_IN as u32) << GRSTCTL_TXFNUM_POS) | GRSTCTL_TXFFLSH,
        );
        while readl(reg(GRSTCTL)) & GRSTCTL_TXFFLSH != 0 {}
    }

    // SAFETY: `TX_BUF` is protected by the surrounding
    // `usb_irq_disable`/_enable window; the ISR cannot preempt.
    if DOUBLE_BUFFER_TX.load(Ordering::Relaxed) {
        unsafe { (*TX_BUF.get()).len = 0 };
    }

    usb_irq_enable();
}

/// USB OTG IRQ handler. Must be called from the interrupt vector.
pub fn irq_handler() {
    let sts = readl(reg(GINTSTS));

    if sts & GINTSTS_RXFLVL != 0 {
        let mask = readl(reg(GINTMSK));
        writel(reg(GINTMSK), mask & !GINTMSK_RXFLVLM);
        let grx = readl(reg(GRXSTSR));
        let ep = grx & GRXSTSP_EPNUM_MSK;
        if ep == 0 {
            cdc::notify_ep0();
        } else {
            cdc::notify_bulk_out();
        }
    }

    if sts & GINTSTS_IEPINT != 0 {
        let daint = readl(reg(DAINT));
        let msk = readl(reg(DAINTMSK));
        let pend = daint & msk;
        writel(reg(DAINTMSK), msk & !daint);
        if pend & (1 << 0) != 0 {
            cdc::notify_ep0();
        }
        if pend & (1 << EP_BULK_IN) != 0 {
            cdc::notify_bulk_in();
            // SAFETY: TX_BUF is accessed from the ISR, which cannot be
            // preempted here. Task-context accesses hold usb_irq_disable
            // across their window, so no overlap is possible.
            if DOUBLE_BUFFER_TX.load(Ordering::Relaxed) {
                unsafe {
                    let tx = &mut *TX_BUF.get();
                    if tx.len != 0 {
                        let ret = fifo_write_packet_fast(EP_BULK_IN, &tx.data, tx.len);
                        if ret == 0 {
                            tx.len = 0;
                        }
                    }
                }
            }
        }
        #[cfg(feature = "trace")]
        if pend & (1 << EP_TRACE_IN) != 0 {
            notify_wake();
        }
    }

    // Order the handler's peripheral accesses before the board wrapper wakes the USB task.
    cortex_m::asm::dmb();
}

/// USB peripheral configuration passed to `init()`.
pub struct OtgConfig {
    /// Base address of the USB OTG peripheral (e.g., 0x5000_0000 for OTG_FS).
    pub base_addr: usize,
    /// IRQ number for the USB OTG interrupt.
    pub irq_num: u16,
    /// Turnaround time for the USB transceiver (TRDT field in GUSBCFG).
    /// Typically 6 for full-speed on STM32F4, 5 for STM32H7.
    pub trdt: u8,
    /// Whether to use VBUS B-session valid override (F446/H7/F7)
    /// instead of NOVBUSSENS (F4 non-F446).
    pub vbus_detection: bool,
    /// Enable double-buffer TX mode for bulk IN.
    pub double_buffer_tx: bool,
}

/// Initialize the USB OTG peripheral.
///
/// This does NOT enable the RCC clock or configure GPIO pins -- the caller
/// must do that before calling this function.
///
/// # Safety
///
/// Must be called exactly once, before any other function in this module.
/// `config.base_addr` must be the valid base address of an STM32 USB OTG peripheral.
pub unsafe fn init(config: &OtgConfig) {
    USB_BASE.store(config.base_addr, Ordering::Relaxed);
    USB_IRQ.store(config.irq_num, Ordering::Relaxed);
    DOUBLE_BUFFER_TX.store(config.double_buffer_tx, Ordering::Relaxed);

    // SAFETY: caller ensures single-call + valid base_addr per function contract.
    unsafe {
        while readl(reg(GRSTCTL)) & GRSTCTL_AHBIDL == 0 {}

        writel(
            reg(GUSBCFG),
            GUSBCFG_FDMOD | GUSBCFG_PHYSEL | ((config.trdt as u32) << GUSBCFG_TRDT_POS),
        );
        let dcfg = readl(reg(DCFG));
        writel(reg(DCFG), dcfg | (3 << DCFG_DSPD_POS));

        if config.vbus_detection {
            writel(reg(GOTGCTL), GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL);
        } else {
            let gccfg = readl(reg(GCCFG));
            writel(reg(GCCFG), gccfg | GCCFG_NOVBUSSENS);
        }

        fifo_configure();

        let mpsize_ep0 = DEPCTL_MPSIZ_16_BYTES;
        writel(reg(diepctl(0)), mpsize_ep0 | DEPCTL_SNAK);
        writel(
            reg(doeptsiz(0)),
            64 | (3 << DOEPTSIZ_STUPCNT_POS) | (1 << DEPTSIZ_PKTCNT_POS),
        );
        writel(reg(doepctl(0)), mpsize_ep0 | DEPCTL_EPENA | DEPCTL_SNAK);

        writel(reg(DIEPMSK), DIEPMSK_XFRCM);
        writel(reg(GINTMSK), GINTMSK_RXFLVLM | GINTMSK_IEPINT);
        writel(reg(GAHBCFG), GAHBCFG_GINT);

        cortex_m::peripheral::NVIC::unmask(IrqNr(config.irq_num));

        let gccfg = readl(reg(GCCFG));
        writel(reg(GCCFG), gccfg | GCCFG_PWRDWN);
        writel(reg(DCTL), 0);
    }
}

/// Check if the bulk IN endpoint is configured (USBAEP bit set).
/// Returns true after SET_CONFIGURATION has been processed.
pub fn is_bulk_in_configured() -> bool {
    let base = USB_BASE.load(Ordering::Relaxed);
    if base == 0 {
        return false;
    }
    let ctl = readl(base + diepctl(EP_BULK_IN));
    ctl & DEPCTL_USBAEP != 0
}

#[cfg(test)]
mod trace_tests {
    #[test]
    #[cfg(feature = "trace")]
    fn accepted_trace_packet_requires_in_completion_wake() {
        assert!(super::trace_packet_needs_completion_wake(1));
        assert!(super::trace_packet_needs_completion_wake(64));
        assert!(!super::trace_packet_needs_completion_wake(0));
        assert!(!super::trace_packet_needs_completion_wake(-1));
    }
}

#[cfg(test)]
mod poll_tests {
    use core::cell::Cell;

    use super::poll_until;

    #[test]
    fn returns_true_once_the_condition_holds() {
        let polls = Cell::new(0);
        assert!(poll_until(10, || {
            polls.set(polls.get() + 1);
            polls.get() == 3
        }));
        assert_eq!(polls.get(), 3);
    }

    #[test]
    fn gives_up_after_the_limit() {
        let polls = Cell::new(0);
        assert!(!poll_until(10, || {
            polls.set(polls.get() + 1);
            false
        }));
        assert_eq!(polls.get(), 10);
    }
}

#[cfg(test)]
mod source_contract_tests {
    fn function_body<'a>(source: &'a str, signature: &str) -> &'a str {
        let signature_start = source
            .find(signature)
            .expect("function signature must exist");
        let body_start = source[signature_start..]
            .find('{')
            .map(|offset| signature_start + offset)
            .expect("function body must open");
        let mut depth = 0usize;

        for (offset, byte) in source.as_bytes()[body_start..].iter().enumerate() {
            match byte {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &source[body_start + 1..body_start + offset];
                    }
                }
                _ => {}
            }
        }

        panic!("function body must close");
    }

    #[test]
    fn irq_handler_ends_with_data_memory_barrier() {
        let source = include_str!("otg.rs");
        let body = function_body(source, "pub fn irq_handler()");
        let terminal_statement = body
            .lines()
            .rev()
            .map(str::trim)
            .find(|line| !line.is_empty() && !line.starts_with("//"));

        assert_eq!(terminal_statement, Some("cortex_m::asm::dmb();"));
    }
}

#[cfg(test)]
mod halt_clear_tests {
    use super::{
        DEPCTL_EPENA, DEPCTL_SD0PID, DEPCTL_STALL, DEPCTL_USBAEP, halt_clear_control_value,
    };

    const MPSIZ_64: u32 = 64;

    #[test]
    fn inactive_endpoint_is_refused() {
        assert_eq!(halt_clear_control_value(MPSIZ_64 | DEPCTL_STALL), None);
    }

    #[test]
    fn active_endpoint_clears_stall_and_resets_data_toggle() {
        let ctl = MPSIZ_64 | DEPCTL_USBAEP | DEPCTL_EPENA | DEPCTL_STALL;
        assert_eq!(
            halt_clear_control_value(ctl),
            Some(MPSIZ_64 | DEPCTL_USBAEP | DEPCTL_EPENA | DEPCTL_SD0PID)
        );
    }

    #[test]
    fn active_unhalted_endpoint_still_resets_data_toggle() {
        let ctl = MPSIZ_64 | DEPCTL_USBAEP;
        assert_eq!(
            halt_clear_control_value(ctl),
            Some(MPSIZ_64 | DEPCTL_USBAEP | DEPCTL_SD0PID)
        );
    }
}

#[cfg(test)]
mod ep0_gate_tests {
    use super::*;

    const DATA_OUT: u32 = 2;
    const SETUP_DATA: u32 = 6;

    fn rx_entry(pktsts: u32) -> u32 {
        (pktsts << GRXSTSP_PKTSTS_POS) | (1 << GRXSTSP_BCNT_POS)
    }

    #[test]
    fn ep0_read_with_empty_rx_queue_is_not_ready() {
        assert_eq!(classify_ep0_read(0), Err(EpError::NotReady));
    }

    #[test]
    fn ep0_read_meeting_a_setup_packet_is_interrupted() {
        assert_eq!(
            classify_ep0_read(rx_entry(SETUP_DATA)),
            Err(EpError::Interrupted)
        );
    }

    #[test]
    fn ep0_read_of_a_data_packet_is_accepted() {
        assert_eq!(classify_ep0_read(rx_entry(DATA_OUT)), Ok(()));
    }

    #[test]
    fn ep0_send_with_a_packet_waiting_is_interrupted() {
        assert_eq!(
            classify_ep0_send(rx_entry(SETUP_DATA), 0),
            Err(EpError::Interrupted)
        );
    }

    #[test]
    fn ep0_send_with_a_packet_waiting_is_interrupted_even_when_in_busy() {
        assert_eq!(
            classify_ep0_send(rx_entry(SETUP_DATA), DEPCTL_EPENA),
            Err(EpError::Interrupted)
        );
    }

    #[test]
    fn ep0_send_while_the_in_endpoint_is_enabled_is_not_ready() {
        assert_eq!(classify_ep0_send(0, DEPCTL_EPENA), Err(EpError::NotReady));
    }

    #[test]
    fn ep0_send_on_an_idle_endpoint_is_accepted() {
        assert_eq!(classify_ep0_send(0, 0), Ok(()));
    }
}
