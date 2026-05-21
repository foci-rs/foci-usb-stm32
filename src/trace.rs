#![cfg(feature = "trace")]

use crate::ep::EP_TRACE_IN_SIZE;
use crate::otg;

/// Return the number of bytes from `data` that fit in one trace bulk packet.
pub fn trace_chunk_len(data: &[u8]) -> usize {
    data.len().min(EP_TRACE_IN_SIZE)
}

/// Send at most one full-speed trace bulk packet.
///
/// Returns the number of bytes accepted by the USB controller, or -1 if the
/// trace endpoint is busy and the caller should retry later.
pub fn send_trace_in(data: &[u8]) -> i8 {
    let len = trace_chunk_len(data);
    otg::usb_send_trace_in(&data[..len])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunk_len_is_limited_to_one_full_speed_packet() {
        assert_eq!(trace_chunk_len(&[0u8; 80]), 64);
        assert_eq!(trace_chunk_len(&[0u8; 16]), 16);
    }
}
