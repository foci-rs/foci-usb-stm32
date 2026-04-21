//! Interior-mutability helper used in place of `static mut` for data
//! whose access discipline is enforced by context (single RTIC task, ISR,
//! or IRQ-disable windows) rather than by the borrow checker.
//!
//! This is equivalent to nightly's `core::cell::SyncUnsafeCell`; we define
//! a local copy to keep the crate on stable toolchains.

use core::cell::UnsafeCell;

/// `Sync`-qualified wrapper around `UnsafeCell<T>`.
///
/// Callers must uphold the usual `UnsafeCell` exclusivity requirements:
/// at most one live reference (shared or exclusive) across all execution
/// contexts that may touch the value. The klipper-usb crate enforces this
/// via RTIC task context ownership or via `usb_irq_disable`/`usb_irq_enable`
/// windows documented at each call site.
#[repr(transparent)]
pub(crate) struct RacyCell<T>(UnsafeCell<T>);

// SAFETY: `RacyCell<T>` is a `Sync`-qualified `UnsafeCell<T>`. The safety
// obligation — at most one live reference across contexts — is discharged
// at each unsafe access site within this crate. `T` is not required to be
// `Send`; any `T` that the crate stores (including raw pointers) is moved
// between tasks only through the documented access protocol.
unsafe impl<T> Sync for RacyCell<T> {}

impl<T> RacyCell<T> {
    /// Construct a new cell holding `value`.
    pub(crate) const fn new(value: T) -> Self {
        Self(UnsafeCell::new(value))
    }

    /// Raw mutable pointer to the inner value.
    ///
    /// The caller must ensure exclusive access for the duration of any
    /// dereference. See the crate-level access protocol.
    pub(crate) const fn get(&self) -> *mut T {
        self.0.get()
    }
}
