// USB descriptor types and static tables (port of usb_cdc.c descriptor section)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Morton Jonuschat
//
// This file may be distributed under the terms of the GNU GPLv3 license.

use crate::ep::{
    EP_ACM, EP_ACM_SIZE, EP_BULK_IN, EP_BULK_IN_SIZE, EP_BULK_OUT, EP_BULK_OUT_SIZE, EP0_SIZE,
};
#[cfg(feature = "trace")]
use crate::ep::{EP_TRACE_IN, EP_TRACE_IN_SIZE};

// USB standard constants
pub const USB_DIR_OUT: u8 = 0x00;
pub const USB_DIR_IN: u8 = 0x80;

pub const USB_REQ_GET_DESCRIPTOR: u8 = 0x06;
pub const USB_REQ_SET_ADDRESS: u8 = 0x05;
pub const USB_REQ_SET_CONFIGURATION: u8 = 0x09;

pub const USB_DT_DEVICE: u8 = 0x01;
pub const USB_DT_CONFIG: u8 = 0x02;
pub const USB_DT_STRING: u8 = 0x03;
pub const USB_DT_INTERFACE: u8 = 0x04;
pub const USB_DT_ENDPOINT: u8 = 0x05;

pub const USB_CLASS_COMM: u8 = 0x02;
#[cfg(feature = "trace")]
pub const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xFF;

pub const USB_ENDPOINT_XFER_BULK: u8 = 0x02;
pub const USB_ENDPOINT_XFER_INT: u8 = 0x03;

pub const USB_LANGID_ENGLISH_US: u16 = 0x0409;

// CDC class constants
pub const USB_CDC_SUBCLASS_ACM: u8 = 0x02;
pub const USB_CDC_ACM_PROTO_AT_V25TER: u8 = 0x01;
pub const USB_CDC_CS_INTERFACE: u8 = 0x24;
pub const USB_CDC_HEADER_TYPE: u8 = 0x00;
pub const USB_CDC_ACM_TYPE: u8 = 0x02;
pub const USB_CDC_UNION_TYPE: u8 = 0x06;

pub const USB_CDC_REQ_SET_LINE_CODING: u8 = 0x20;
pub const USB_CDC_REQ_GET_LINE_CODING: u8 = 0x21;
pub const USB_CDC_REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

// String descriptor IDs
pub const USB_STR_ID_MANUFACTURER: u8 = 1;
pub const USB_STR_ID_PRODUCT: u8 = 2;
pub const USB_STR_ID_SERIAL: u8 = 3;

/// USB control request (8 bytes, matches struct usb_ctrlrequest)
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct UsbCtrlRequest {
    pub b_request_type: u8,
    pub b_request: u8,
    pub w_value: u16,
    pub w_index: u16,
    pub w_length: u16,
}

/// CDC line coding (7 bytes, matches struct usb_cdc_line_coding)
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct UsbCdcLineCoding {
    pub dw_dte_rate: u32,
    pub b_char_format: u8,
    pub b_parity_type: u8,
    pub b_data_bits: u8,
}

/// USB device descriptor (18 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbDeviceDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub bcd_usb: u16,
    pub b_device_class: u8,
    pub b_device_sub_class: u8,
    pub b_device_protocol: u8,
    pub b_max_packet_size0: u8,
    pub id_vendor: u16,
    pub id_product: u16,
    pub bcd_device: u16,
    pub i_manufacturer: u8,
    pub i_product: u8,
    pub i_serial_number: u8,
    pub b_num_configurations: u8,
}

/// USB config descriptor (9 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbConfigDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub w_total_length: u16,
    pub b_num_interfaces: u8,
    pub b_configuration_value: u8,
    pub i_configuration: u8,
    pub bm_attributes: u8,
    pub b_max_power: u8,
}

/// USB interface descriptor (9 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbInterfaceDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_interface_number: u8,
    pub b_alternate_setting: u8,
    pub b_num_endpoints: u8,
    pub b_interface_class: u8,
    pub b_interface_sub_class: u8,
    pub b_interface_protocol: u8,
    pub i_interface: u8,
}

/// USB endpoint descriptor (7 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbEndpointDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_endpoint_address: u8,
    pub bm_attributes: u8,
    pub w_max_packet_size: u16,
    pub b_interval: u8,
}

/// CDC header descriptor (5 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbCdcHeaderDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_descriptor_sub_type: u8,
    pub bcd_cdc: u16,
}

/// CDC ACM descriptor (4 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbCdcAcmDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_descriptor_sub_type: u8,
    pub bm_capabilities: u8,
}

/// CDC union descriptor (5 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct UsbCdcUnionDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_descriptor_sub_type: u8,
    pub b_master_interface0: u8,
    pub b_slave_interface0: u8,
}

/// Full CDC configuration descriptor (matches struct config_s in usb_cdc.c)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct CdcConfigDescriptor {
    pub config: UsbConfigDescriptor,
    pub iface0: UsbInterfaceDescriptor,
    pub cdc_hdr: UsbCdcHeaderDescriptor,
    pub cdc_acm: UsbCdcAcmDescriptor,
    pub cdc_union: UsbCdcUnionDescriptor,
    pub ep1: UsbEndpointDescriptor,
    pub iface1: UsbInterfaceDescriptor,
    pub ep2: UsbEndpointDescriptor,
    pub ep3: UsbEndpointDescriptor,
}

/// CDC configuration descriptor plus trace vendor-specific bulk IN interface.
#[cfg(feature = "trace")]
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct CdcTraceConfigDescriptor {
    pub config: UsbConfigDescriptor,
    pub iface0: UsbInterfaceDescriptor,
    pub cdc_hdr: UsbCdcHeaderDescriptor,
    pub cdc_acm: UsbCdcAcmDescriptor,
    pub cdc_union: UsbCdcUnionDescriptor,
    pub ep1: UsbEndpointDescriptor,
    pub iface1: UsbInterfaceDescriptor,
    pub ep2: UsbEndpointDescriptor,
    pub ep3: UsbEndpointDescriptor,
    pub trace_iface: UsbInterfaceDescriptor,
    pub trace_ep: UsbEndpointDescriptor,
}

/// Build the static device descriptor from config parameters.
pub const fn build_device_descriptor(vid: u16, pid: u16) -> UsbDeviceDescriptor {
    UsbDeviceDescriptor {
        b_length: core::mem::size_of::<UsbDeviceDescriptor>() as u8,
        b_descriptor_type: USB_DT_DEVICE,
        bcd_usb: 0x0200_u16.to_le(),
        b_device_class: USB_CLASS_COMM,
        b_device_sub_class: 0,
        b_device_protocol: 0,
        b_max_packet_size0: EP0_SIZE as u8,
        id_vendor: vid.to_le(),
        id_product: pid.to_le(),
        bcd_device: 0x0100_u16.to_le(),
        i_manufacturer: USB_STR_ID_MANUFACTURER,
        i_product: USB_STR_ID_PRODUCT,
        i_serial_number: USB_STR_ID_SERIAL,
        b_num_configurations: 1,
    }
}

/// Build the static CDC configuration descriptor.
pub const fn build_config_descriptor() -> CdcConfigDescriptor {
    CdcConfigDescriptor {
        config: UsbConfigDescriptor {
            b_length: core::mem::size_of::<UsbConfigDescriptor>() as u8,
            b_descriptor_type: USB_DT_CONFIG,
            w_total_length: (core::mem::size_of::<CdcConfigDescriptor>() as u16).to_le(),
            b_num_interfaces: 2,
            b_configuration_value: 1,
            i_configuration: 0,
            bm_attributes: 0xC0,
            b_max_power: 50,
        },
        iface0: UsbInterfaceDescriptor {
            b_length: core::mem::size_of::<UsbInterfaceDescriptor>() as u8,
            b_descriptor_type: USB_DT_INTERFACE,
            b_interface_number: 0,
            b_alternate_setting: 0,
            b_num_endpoints: 1,
            b_interface_class: USB_CLASS_COMM,
            b_interface_sub_class: USB_CDC_SUBCLASS_ACM,
            b_interface_protocol: USB_CDC_ACM_PROTO_AT_V25TER,
            i_interface: 0,
        },
        cdc_hdr: UsbCdcHeaderDescriptor {
            b_length: core::mem::size_of::<UsbCdcHeaderDescriptor>() as u8,
            b_descriptor_type: USB_CDC_CS_INTERFACE,
            b_descriptor_sub_type: USB_CDC_HEADER_TYPE,
            bcd_cdc: 0x0110_u16.to_le(),
        },
        cdc_acm: UsbCdcAcmDescriptor {
            b_length: core::mem::size_of::<UsbCdcAcmDescriptor>() as u8,
            b_descriptor_type: USB_CDC_CS_INTERFACE,
            b_descriptor_sub_type: USB_CDC_ACM_TYPE,
            bm_capabilities: 0x06,
        },
        cdc_union: UsbCdcUnionDescriptor {
            b_length: core::mem::size_of::<UsbCdcUnionDescriptor>() as u8,
            b_descriptor_type: USB_CDC_CS_INTERFACE,
            b_descriptor_sub_type: USB_CDC_UNION_TYPE,
            b_master_interface0: 0,
            b_slave_interface0: 1,
        },
        ep1: UsbEndpointDescriptor {
            b_length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
            b_descriptor_type: USB_DT_ENDPOINT,
            b_endpoint_address: EP_ACM as u8 | USB_DIR_IN,
            bm_attributes: USB_ENDPOINT_XFER_INT,
            w_max_packet_size: (EP_ACM_SIZE as u16).to_le(),
            b_interval: 255,
        },
        iface1: UsbInterfaceDescriptor {
            b_length: core::mem::size_of::<UsbInterfaceDescriptor>() as u8,
            b_descriptor_type: USB_DT_INTERFACE,
            b_interface_number: 1,
            b_alternate_setting: 0,
            b_num_endpoints: 2,
            b_interface_class: 0x0A,
            b_interface_sub_class: 0,
            b_interface_protocol: 0,
            i_interface: 0,
        },
        ep2: UsbEndpointDescriptor {
            b_length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
            b_descriptor_type: USB_DT_ENDPOINT,
            b_endpoint_address: EP_BULK_OUT as u8,
            bm_attributes: USB_ENDPOINT_XFER_BULK,
            w_max_packet_size: (EP_BULK_OUT_SIZE as u16).to_le(),
            b_interval: 0,
        },
        ep3: UsbEndpointDescriptor {
            b_length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
            b_descriptor_type: USB_DT_ENDPOINT,
            b_endpoint_address: EP_BULK_IN as u8 | USB_DIR_IN,
            bm_attributes: USB_ENDPOINT_XFER_BULK,
            w_max_packet_size: (EP_BULK_IN_SIZE as u16).to_le(),
            b_interval: 0,
        },
    }
}

/// Build the CDC plus trace configuration descriptor.
#[cfg(feature = "trace")]
pub const fn build_trace_config_descriptor() -> CdcTraceConfigDescriptor {
    let cdc = build_config_descriptor();
    CdcTraceConfigDescriptor {
        config: UsbConfigDescriptor {
            w_total_length: (core::mem::size_of::<CdcTraceConfigDescriptor>() as u16).to_le(),
            b_num_interfaces: 3,
            ..cdc.config
        },
        iface0: cdc.iface0,
        cdc_hdr: cdc.cdc_hdr,
        cdc_acm: cdc.cdc_acm,
        cdc_union: cdc.cdc_union,
        ep1: cdc.ep1,
        iface1: cdc.iface1,
        ep2: cdc.ep2,
        ep3: cdc.ep3,
        trace_iface: UsbInterfaceDescriptor {
            b_length: core::mem::size_of::<UsbInterfaceDescriptor>() as u8,
            b_descriptor_type: USB_DT_INTERFACE,
            b_interface_number: 2,
            b_alternate_setting: 0,
            b_num_endpoints: 1,
            b_interface_class: USB_CLASS_VENDOR_SPECIFIC,
            b_interface_sub_class: 0,
            b_interface_protocol: 0,
            i_interface: 0,
        },
        trace_ep: UsbEndpointDescriptor {
            b_length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
            b_descriptor_type: USB_DT_ENDPOINT,
            b_endpoint_address: EP_TRACE_IN as u8 | USB_DIR_IN,
            bm_attributes: USB_ENDPOINT_XFER_BULK,
            w_max_packet_size: (EP_TRACE_IN_SIZE as u16).to_le(),
            b_interval: 0,
        },
    }
}

/// A USB string descriptor entry: header (2 bytes) + UTF-16LE data.
/// Maximum string length is 31 UTF-16 code units (62 bytes of data + 2 byte header = 64 bytes).
pub struct StringDescriptorBuf {
    buf: [u8; 64],
    len: u8,
}

impl StringDescriptorBuf {
    /// Create a string descriptor from a static ASCII string.
    /// Panics if the string exceeds 31 characters.
    pub const fn from_ascii(s: &str) -> Self {
        let bytes = s.as_bytes();
        assert!(
            bytes.len() <= 31,
            "USB string descriptor too long (max 31 chars)"
        );
        let data_len = bytes.len() * 2;
        let total_len = 2 + data_len;
        let mut buf = [0u8; 64];
        buf[0] = total_len as u8;
        buf[1] = USB_DT_STRING;
        let mut i = 0;
        while i < bytes.len() {
            buf[2 + i * 2] = bytes[i];
            buf[2 + i * 2 + 1] = 0;
            i += 1;
        }
        Self {
            buf,
            len: total_len as u8,
        }
    }

    /// Create the language ID descriptor (string index 0).
    pub const fn lang_ids() -> Self {
        let mut buf = [0u8; 64];
        buf[0] = 4; // bLength
        buf[1] = USB_DT_STRING;
        // USB_LANGID_ENGLISH_US = 0x0409
        buf[2] = 0x09;
        buf[3] = 0x04;
        Self { buf, len: 4 }
    }

    /// Get the descriptor bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// Get the total length of the descriptor.
    pub fn len(&self) -> u8 {
        self.len
    }

    /// Returns true if the descriptor has no string data (only the 2-byte header).
    pub fn is_empty(&self) -> bool {
        self.len <= 2
    }

    /// Get a pointer to the buffer data and the length, without creating a
    /// reference. Used for accessing string descriptors stored in mutable
    /// statics via raw pointers.
    ///
    /// # Safety
    ///
    /// The caller must ensure no mutable reference to this `StringDescriptorBuf`
    /// exists when the returned pointer is dereferenced.
    pub unsafe fn raw_ptr_and_len(this: *const Self) -> (*const u8, u8) {
        // SAFETY: caller upholds the safety contract: no live mutable
        // reference to *this while the returned pointer is used.
        unsafe {
            let buf_ptr = core::ptr::addr_of!((*this).buf) as *const u8;
            let len = core::ptr::addr_of!((*this).len).read();
            (buf_ptr, len)
        }
    }
}

/// Fill a string descriptor buffer from a chip ID (hex nibbles).
/// Matches Klipper's `usb_fill_serial()`.
///
/// `id` is the raw chip ID bytes. `strlen` is the number of hex nibbles
/// to encode (typically `id.len() * 2`).
pub fn fill_serial_from_chip_id(buf: &mut StringDescriptorBuf, strlen: usize, id: &[u8]) {
    let data_len = strlen * 2;
    let total_len = 2 + data_len;
    assert!(total_len <= 64, "Serial descriptor too long");
    buf.buf[0] = total_len as u8;
    buf.buf[1] = USB_DT_STRING;
    for i in 0..strlen {
        let c = if i & 1 != 0 {
            id[i / 2] & 0x0F
        } else {
            id[i / 2] >> 4
        };
        let ch = if c < 10 { c + b'0' } else { c - 10 + b'A' };
        buf.buf[2 + i * 2] = ch;
        buf.buf[2 + i * 2 + 1] = 0;
    }
    buf.len = total_len as u8;
}

/// Descriptor lookup entry (matches struct descriptor_s in usb_cdc.c).
pub struct DescriptorEntry {
    pub w_value: u16,
    pub w_index: u16,
    pub data: *const u8,
    pub size: u8,
}

// SAFETY: DescriptorEntry contains a raw pointer to static data that is
// initialized once and never modified. All descriptor data lives in static
// storage with 'static lifetime.
unsafe impl Send for DescriptorEntry {}
unsafe impl Sync for DescriptorEntry {}

#[cfg(test)]
mod trace_descriptor_tests {
    use super::*;

    fn bytes<T>(value: &T) -> &[u8] {
        let ptr = value as *const T as *const u8;
        // SAFETY: `value` is valid for `size_of::<T>()` bytes and this helper
        // only reinterprets it as immutable bytes for descriptor layout tests.
        unsafe { core::slice::from_raw_parts(ptr, core::mem::size_of::<T>()) }
    }

    #[test]
    fn production_config_descriptor_bytes_stay_stable() {
        let cfg = build_config_descriptor();
        let data = bytes(&cfg);
        let total_length = cfg.config.w_total_length;

        assert_eq!(data.len(), core::mem::size_of::<CdcConfigDescriptor>());
        assert_eq!(cfg.config.b_num_interfaces, 2);
        assert_eq!(u16::from_le(total_length), data.len() as u16);
        assert_eq!(cfg.iface0.b_interface_number, 0);
        assert_eq!(cfg.iface1.b_interface_number, 1);
    }

    #[cfg(feature = "trace")]
    #[test]
    fn trace_config_adds_vendor_bulk_in_after_cdc() {
        let cfg = build_trace_config_descriptor();
        let data = bytes(&cfg);
        let total_length = cfg.config.w_total_length;
        let trace_ep_address = cfg.trace_ep.b_endpoint_address;

        assert_eq!(cfg.config.b_num_interfaces, 3);
        assert_eq!(u16::from_le(total_length), data.len() as u16);
        assert_eq!(cfg.iface0.b_interface_number, 0);
        assert_eq!(cfg.iface1.b_interface_number, 1);
        assert_eq!(cfg.trace_iface.b_interface_number, 2);
        assert_eq!(cfg.trace_iface.b_interface_class, USB_CLASS_VENDOR_SPECIFIC);
        assert_eq!(trace_ep_address, crate::ep::EP_TRACE_IN as u8 | USB_DIR_IN);
    }
}
