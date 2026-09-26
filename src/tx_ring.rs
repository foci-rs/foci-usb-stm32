use core::sync::atomic::{AtomicU32, Ordering};

use crate::sync::RacyCell;

pub(crate) struct TxRing<const N: usize> {
    buf: RacyCell<[u8; N]>,
    reserved_end: AtomicU32,
    published_end: AtomicU32,
    released_end: AtomicU32,
    open_writers: AtomicU32,
}

impl<const N: usize> TxRing<N> {
    const CAPACITY: u32 = {
        assert!(N.is_power_of_two());
        N as u32
    };

    pub(crate) const fn new() -> Self {
        Self::starting_at(0)
    }

    const fn starting_at(pos: u32) -> Self {
        Self {
            buf: RacyCell::new([0; N]),
            reserved_end: AtomicU32::new(pos),
            published_end: AtomicU32::new(pos),
            released_end: AtomicU32::new(pos),
            open_writers: AtomicU32::new(0),
        }
    }

    #[must_use]
    pub(crate) fn push(&self, data: &[u8]) -> bool {
        self.write_frame(data, || {})
    }

    #[must_use]
    fn write_frame(&self, data: &[u8], before_commit: impl FnOnce()) -> bool {
        self.open_writers.fetch_add(1, Ordering::Relaxed);
        let start = self.reserve(data.len());
        if let Some(start) = start {
            self.copy_in(start, data);
        }
        before_commit();
        let outermost = self.open_writers.fetch_sub(1, Ordering::AcqRel) == 1;
        if outermost {
            self.publish();
        }
        start.is_some()
    }

    fn reserve(&self, len: usize) -> Option<u32> {
        let len = u32::try_from(len).ok()?;
        loop {
            let (start, used) = self.consistent_usage();
            if Self::CAPACITY - used < len {
                return None;
            }
            if self
                .reserved_end
                .compare_exchange_weak(
                    start,
                    start.wrapping_add(len),
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                return Some(start);
            }
        }
    }

    fn consistent_usage(&self) -> (u32, u32) {
        loop {
            let released_end = self.released_end.load(Ordering::Acquire);
            let reserved_end = self.reserved_end.load(Ordering::Relaxed);
            let used = reserved_end.wrapping_sub(released_end);
            if used <= Self::CAPACITY {
                return (reserved_end, used);
            }
        }
    }

    fn copy_in(&self, start: u32, data: &[u8]) {
        let index = start as usize % N;
        let first = data.len().min(N - index);
        let base = self.buf.get() as *mut u8;
        // SAFETY: `reserve` handed this producer exclusive ownership of
        // `[start, start + data.len())`; the consumer does not read it before
        // `publish` and no other producer is given an overlapping range.
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(index), first);
            core::ptr::copy_nonoverlapping(data.as_ptr().add(first), base, data.len() - first);
        }
    }

    fn publish(&self) {
        let reserved_end = self.reserved_end.load(Ordering::Relaxed);
        let _ = self.published_end.fetch_update(
            Ordering::Release,
            Ordering::Relaxed,
            |published_end| {
                ((reserved_end.wrapping_sub(published_end) as i32) > 0).then_some(reserved_end)
            },
        );
    }

    pub(crate) fn pending(&self) -> usize {
        let published_end = self.published_end.load(Ordering::Acquire);
        published_end.wrapping_sub(self.released_end.load(Ordering::Relaxed)) as usize
    }

    pub(crate) fn peek(&self, out: &mut [u8]) -> usize {
        let len = out.len().min(self.pending());
        let index = self.released_end.load(Ordering::Relaxed) as usize % N;
        let first = len.min(N - index);
        let base = self.buf.get() as *const u8;
        // SAFETY: `[released_end, released_end + len)` is published, so producers
        // have finished writing it and will not reuse it until `consume` releases it.
        unsafe {
            core::ptr::copy_nonoverlapping(base.add(index), out.as_mut_ptr(), first);
            core::ptr::copy_nonoverlapping(base, out.as_mut_ptr().add(first), len - first);
        }
        len
    }

    pub(crate) fn consume(&self, len: usize) {
        let released_end = self.released_end.load(Ordering::Relaxed);
        self.released_end
            .store(released_end.wrapping_add(len as u32), Ordering::Release);
    }
}

/// Length of the next bulk IN packet for `pending` staged bytes.
///
/// Never sends exactly one full final packet, which would need a trailing
/// zero-length packet to end the transfer.
pub(crate) fn bulk_in_packet_len(pending: usize, max_packet: usize) -> usize {
    match pending.cmp(&max_packet) {
        core::cmp::Ordering::Less => pending,
        core::cmp::Ordering::Equal => max_packet - 1,
        core::cmp::Ordering::Greater => max_packet,
    }
}

#[cfg(test)]
mod tests {
    use super::{TxRing, bulk_in_packet_len};

    fn drain(ring: &TxRing<16>) -> ([u8; 16], usize) {
        let mut out = [0u8; 16];
        let n = ring.peek(&mut out);
        ring.consume(n);
        (out, n)
    }

    #[test]
    fn frames_read_back_in_order_and_free_space_on_consume() {
        let ring = TxRing::<16>::new();

        assert!(ring.push(b"abcdef"));
        assert!(ring.push(b"ghij"));
        assert_eq!(ring.pending(), 10);

        let (out, n) = drain(&ring);
        assert_eq!(&out[..n], b"abcdefghij");
        assert_eq!(ring.pending(), 0);
        assert!(ring.push(&[7u8; 16]));
    }

    #[test]
    fn frame_that_does_not_fit_is_rejected_whole() {
        let ring = TxRing::<16>::new();
        assert!(ring.push(&[1u8; 12]));

        assert!(!ring.push(&[2u8; 5]));

        assert_eq!(ring.pending(), 12);
        let (out, n) = drain(&ring);
        assert_eq!(&out[..n], &[1u8; 12]);
    }

    #[test]
    fn frame_wrapping_the_storage_end_reads_back_intact() {
        let ring = TxRing::<16>::new();
        assert!(ring.push(&[0u8; 12]));
        drain(&ring);

        assert!(ring.push(b"0123456789"));

        let mut out = [0u8; 16];
        let n = ring.peek(&mut out);
        assert_eq!(&out[..n], b"0123456789");
    }

    #[test]
    fn partial_peek_consumes_only_what_was_sent() {
        let ring = TxRing::<16>::new();
        assert!(ring.push(b"abcdefgh"));

        let mut out = [0u8; 3];
        assert_eq!(ring.peek(&mut out), 3);
        ring.consume(3);

        let (rest, n) = drain(&ring);
        assert_eq!(&rest[..n], b"defgh");
    }

    #[test]
    fn preempting_writer_is_published_only_after_the_outer_frame() {
        let ring = TxRing::<16>::new();

        assert!(ring.write_frame(b"outer", || {
            assert!(ring.push(b"inner"));
            assert_eq!(ring.pending(), 0);
        }));

        let (out, n) = drain(&ring);
        assert_eq!(&out[..n], b"outerinner");
    }

    #[test]
    fn rejected_preempting_writer_leaves_outer_frame_publishable() {
        let ring = TxRing::<16>::new();

        assert!(ring.write_frame(&[1u8; 12], || {
            assert!(!ring.push(&[2u8; 8]));
            assert_eq!(ring.pending(), 0);
        }));

        assert_eq!(ring.pending(), 12);
    }

    #[test]
    fn positions_wrap_past_u32_max() {
        let ring = TxRing::<16>::starting_at(u32::MAX - 4);

        assert!(ring.push(b"abcdefghij"));
        assert_eq!(ring.pending(), 10);
        let (out, n) = drain(&ring);
        assert_eq!(&out[..n], b"abcdefghij");

        assert!(ring.push(&[3u8; 16]));
        assert!(!ring.push(&[4u8; 1]));
    }

    #[test]
    fn packet_length_avoids_an_exactly_full_final_packet() {
        assert_eq!(bulk_in_packet_len(10, 64), 10);
        assert_eq!(bulk_in_packet_len(64, 64), 63);
        assert_eq!(bulk_in_packet_len(65, 64), 64);
        assert_eq!(bulk_in_packet_len(2048, 64), 64);
    }
}
